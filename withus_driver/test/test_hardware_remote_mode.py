#!/usr/bin/env python3
"""Exercise the continuous 0x501 request profile on an existing virtual CAN.

Source this workspace's install/setup.bash before running this file. No CAN
interface is created or configured; the kernel link kind must be vcan. The
synthetic peer reports mode 3 while A is selected and request 1 is present,
matching the observed semi-automatic mode value. This verifies driver software;
it does not establish physical control ownership, braking, or robot behavior.

Only this process's own driver child is signalled or killed during cleanup.
The adjacent hardware_mode_only test supplies shared socket and ROS helpers.
"""

import argparse
import os
from pathlib import Path
import statistics
import subprocess
import tempfile
import time

os.environ.setdefault("ROS_DOMAIN_ID", "180")
os.environ["ROS_LOCALHOST_ONLY"] = "1"
os.environ.setdefault("ROS_LOG_DIR", tempfile.mkdtemp(prefix="withus-remote-mode-ros-"))

import rclpy
from ament_index_python.packages import get_package_prefix
from withus_msgs.msg import BodyCommand, DriveCommand, DriverStatus

from test_hardware_mode_only import (
    CAN_FRAME, FEEDBACK_IDS, Harness, Observation, Peer, RELEASE_SECONDS,
    require_vcan,
)


class RemotePeer(Peer):
    """Feedback source; no steering, brake, speed, or body command is executed."""

    def __init__(self, interface):
        super().__init__(interface)
        self.values.update(a_raw=0, ad_mode=3)

    def healthy(self, selected=False):
        with self.lock:
            self.values.update(a_raw=int(selected), ad_mode=3, paused=set(),
                               deny_ad=False, refuse_remote=False, mode_override=None,
                               gear=2, speed=0.0, fault=False)
            self.mode = 0

    def _receive(self):
        while True:
            try:
                packet = self.sock.recv(CAN_FRAME.size)
            except BlockingIOError:
                return
            if len(packet) != CAN_FRAME.size:
                continue
            can_id, length, data = CAN_FRAME.unpack(packet)
            with self.lock:
                self.observations.append(Observation(time.monotonic(), can_id, length, data))
                if can_id != 0x501 or length != 8:
                    continue
                request = data[0] & 15
                if request == 1 and self.values["a_raw"] == 1 and not self.values["deny_ad"]:
                    self.mode = self.values["ad_mode"]
                elif (request == 0 or self.values["a_raw"] == 0) and not self.values["refuse_remote"]:
                    self.mode = 0


class RemoteHarness(Harness):
    def __init__(self, executable, peer):
        self.states_seen = set()
        super().__init__(executable, peer)

    def _status(self, msg):
        super()._status(msg)
        self.states_seen.add(msg.state)

    def start(self, enabled):
        self.status = None
        self.vehicle = None
        self.states_seen.clear()
        fd, path = tempfile.mkstemp(prefix="withus-remote-mode-driver-", suffix=".log")
        self.log = os.fdopen(fd, "w")
        self.logs.append(path)
        command = [str(self.executable), "--ros-args", "-r", "__ns:=" + self.namespace,
                   "-p", "can_interface:=" + self.peer.interface,
                   "-p", "enable_tx:=" + str(enabled).lower(),
                   "-p", "control_profile:=hardware_remote_mode",
                   "-p", "release_timeout_ms:=300",
                   "-p", "control_request_timeout_ms:=500"]
        self.process = subprocess.Popen(command, stdout=self.log, stderr=subprocess.STDOUT)
        # No received CAN feedback is necessary for the initial request.
        self.wait(lambda: self.status is not None and self.vehicle is not None,
                  "driver status topics arrive", 5.0)
        self.wait(self.service.service_is_ready, "optional SetControl service discovered")
        self.wait(lambda: self.drive.get_subscription_count() > 0 and
                  self.body.get_subscription_count() > 0, "command subscriptions discovered")
        self.check(self.status.control_profile == "hardware_remote_mode",
                   "DriverStatus identifies hardware_remote_mode profile")
        self.pump(0.1)

    def heartbeat(self, since, minimum=4):
        frames = self.peer.history(since)
        self.check(len(frames) >= minimum and all((f.data[0] & 15) == 1 for f in frames),
                   "periodic request 1 remains asserted without an intervening request 0")
        self.check(self.status.state == DriverStatus.MONITORING and self.status.control_requested,
                   "driver reports its request without claiming ACTIVE control ownership")

    def reject_motion(self):
        for cls, publisher, label in ((DriveCommand, self.drive, "DriveCommand"),
                                      (BodyCommand, self.body, "BodyCommand")):
            message = cls()
            message.header.frame_id = "base_link"
            message.header.stamp = self.node.get_clock().now().to_msg()
            message.session_id = self.status.session_id
            if cls is DriveCommand:
                message.gear = 1
                message.speed_mps = 0.2
                message.steering_angle_rad = 0.1
            else:
                message.headlight_on = True
                message.horn_on = True
            before = self.status.rejected_commands
            publisher.publish(message)
            self.wait(lambda: self.status.rejected_commands > before,
                      label + " rejected in the request-only profile")

    def check_shutdown_release(self):
        begin = time.monotonic()
        self.stop()
        frames = self.peer.history(begin)
        zeros = [f for f in frames if (f.data[0] & 15) == 0]
        self.check(len(zeros) >= 8 and zeros[-1].at - zeros[0].at >= RELEASE_SECONDS - 0.08,
                   "SIGINT preserves the repeated request-0 release phase")
        self.check(time.monotonic() - begin < 2.0, "SIGINT release completes within a bounded interval")
        count = len(self.peer.history())
        self.pump(0.12)
        self.check(len(self.peer.history()) == count, "no frames follow the owned driver's shutdown")
        self.only_501(begin)


def run(h):
    begin = time.monotonic()
    h.start(False)
    h.control(True, accepted=False)
    h.check(not h.status.control_requested and not h.peer.history(begin),
            "enable_tx=false suppresses every command frame")
    h.stop()

    # Immediately request control, including before any CAN feedback arrives.
    h.peer.inject(paused=set(FEEDBACK_IDS))
    counter_start = time.monotonic()
    h.start(True)
    h.pump(0.65)
    h.check(not h.vehicle.status_10a_received and not h.vehicle.status_301_received and
            not h.vehicle.status_303_received and not h.vehicle.status_304_received,
            "missing startup feedback remains explicitly unreceived")
    h.heartbeat(counter_start, minimum=25)
    frames = h.peer.history(counter_start)
    intervals = [b.at - a.at for a, b in zip(frames, frames[1:])]
    h.check(0.01 < statistics.median(intervals) < 0.035, "request cadence is near 20 ms")

    # A and mode are observations, including the robot's mode 3 feedback.
    h.peer.inject(paused=set())
    begin = time.monotonic()
    for a_raw, mode in ((0, 0), (1, 3), (0, 0), (1, 3)):
        h.peer.inject(a_raw=a_raw)
        h.wait(lambda: h.vehicle.status_10a_valid and h.vehicle.status_303_valid and
               h.vehicle.remote_a_raw == a_raw and h.vehicle.drive_mode == mode,
               "A={} and actual mode={} are reported unchanged".format(a_raw, mode))
        h.pump(0.08)
    h.heartbeat(begin)
    h.reject_motion()

    # None of these observations may silently change the request bit. Confirm
    # the fields too, so unconditional transmission does not conceal faults.
    scenarios = [
        ("unknown A value", {"a_raw": 2},
         lambda: h.vehicle.remote_a_raw == 2 and h.vehicle.status_10a_valid),
        ("stale A feedback", {"paused": {0x10A}},
         lambda: h.vehicle.status_10a_received and not h.vehicle.status_10a_valid),
        ("stale diagnosis feedback", {"paused": {0x301}},
         lambda: h.vehicle.status_301_received and not h.vehicle.status_301_valid),
        ("reported movement in Drive", {"paused": set(), "a_raw": 1, "gear": 1, "speed": 0.2},
         lambda: h.vehicle.gear == 1 and h.vehicle.speed_mps > 0.1),
        ("reported motor fault", {"fault": True}, lambda: h.vehicle.motor_fault),
        ("mode 1", {"mode_override": 1}, lambda: h.vehicle.drive_mode == 1),
        ("mode 2", {"mode_override": 2}, lambda: h.vehicle.drive_mode == 2),
        ("raw mode 15", {"mode_override": 15}, lambda: h.vehicle.drive_mode == 15),
    ]
    begin = time.monotonic()
    for label, injection, observed in scenarios:
        h.peer.inject(**injection)
        h.wait(observed, label + " remains visible in VehicleStatus")
        h.pump(0.08)
    h.pump(0.55)
    h.heartbeat(begin)
    h.check(h.states_seen == {DriverStatus.MONITORING},
            "A, freshness, gear, speed, fault, and mode do not gate or confirm the heartbeat")
    h.only_501()

    # Optional pause uses a bounded release. Completion is confirmed only by
    # fresh manual-mode feedback received after a successful request-0 write.
    h.peer.healthy(selected=True)
    h.wait(lambda: h.vehicle.drive_mode == 3 and not h.vehicle.motor_fault and
           h.vehicle.status_301_valid and h.vehicle.status_303_valid and h.vehicle.status_304_valid,
           "normal mode-3 feedback restored")
    begin = time.monotonic()
    h.control(False)
    h.wait(lambda: h.status.state == DriverStatus.STOPPING, "pause enters the release phase")
    h.control(True, accepted=False)
    h.control(False)
    h.assert_release(begin, DriverStatus.MONITORING, "fresh manual mode confirms the optional pause")
    h.check(not h.status.control_requested, "paused driver reports no control request")

    # Resume without requiring A, Neutral, fresh feedback, or a separate reset.
    h.peer.inject(paused=set(FEEDBACK_IDS), gear=1, speed=0.2, fault=True)
    h.pump(0.25)
    begin = time.monotonic()
    h.control(True)
    h.pump(0.3)
    h.heartbeat(begin)

    # A refused release is not misreported as confirmed, but still stops at the
    # fixed deadline. Enable resumes directly from this diagnostic fault.
    h.peer.healthy(selected=True)
    h.wait(lambda: h.vehicle.drive_mode == 3 and h.vehicle.status_301_valid and
           h.vehicle.status_303_valid and h.vehicle.status_304_valid,
           "fresh mode 3 returns before the unconfirmed-release test")
    h.peer.inject(refuse_remote=True)
    begin = time.monotonic()
    h.control(False)
    h.wait(lambda: h.status.state == DriverStatus.STOPPING, "unconfirmed pause starts its bounded phase")
    h.pump(0.06)
    h.control(False)
    h.assert_release(begin, DriverStatus.FAULT, "missing manual acknowledgement reports an unconfirmed release")
    h.check(not h.status.control_requested and h.vehicle.drive_mode == 3,
            "unconfirmed pause preserves the actual robot mode without requesting control")
    begin = time.monotonic()
    h.control(True)
    h.pump(0.15)
    h.heartbeat(begin)

    frames = h.peer.history(counter_start)
    h.check(all((b.data[0] >> 4) == ((a.data[0] >> 4) + 1) % 16
                for a, b in zip(frames, frames[1:])),
            "501 counter advances modulo 16 across pauses and resumes")
    h.peer.inject(refuse_remote=False)
    h.check_shutdown_release()

    # A feedback is unnecessary for a fresh 0x303 manual release confirmation.
    h.peer.healthy()
    h.peer.inject(paused={0x10A})
    h.start(True)
    begin = time.monotonic()
    h.pump(0.15)
    h.check(not h.vehicle.status_10a_received, "the final run has no A feedback")
    h.heartbeat(begin)
    h.check_shutdown_release()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interface", default="vcan0")
    parser.add_argument("--driver-prefix", type=Path,
                        help="installed withus_driver package prefix (optional)")
    args = parser.parse_args()
    interface = require_vcan(args.interface)
    prefix = args.driver_prefix or Path(get_package_prefix("withus_driver"))
    executable = prefix / "lib" / "withus_driver" / "withus_driver_node"
    if not executable.is_file():
        raise FileNotFoundError(executable)
    peer = RemotePeer(interface).start()
    harness = None
    rclpy.init()
    try:
        harness = RemoteHarness(executable, peer)
        run(harness)
        print("ALL {} hardware_remote_mode virtual CAN/ROS assertions passed".format(harness.checks), flush=True)
    finally:
        try:
            if harness:
                harness.close()
                print("Driver logs: " + ", ".join(harness.logs), flush=True)
        finally:
            rclpy.shutdown()
            peer.close()


if __name__ == "__main__":
    main()

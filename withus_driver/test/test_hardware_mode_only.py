#!/usr/bin/env python3
"""Exercise hardware_mode_only against a synthetic peer on an existing vcan.

Source this workspace's install/setup.bash, stop other publishers on that vcan,
then run this file. It creates no interfaces and refuses a physical CAN link.
The synthetic acknowledgements verify software behavior, not robot stop safety.
Only this program's own driver child is signalled or killed during cleanup.
"""

import argparse
from collections import deque
from dataclasses import dataclass
import json
import os
from pathlib import Path
import select
import signal
import socket
import statistics
import struct
import subprocess
import tempfile
import threading
import time

# Set before importing/initializing ROS. Use a separate ROS domain and local DDS.
os.environ.setdefault("ROS_DOMAIN_ID", "179")
os.environ["ROS_LOCALHOST_ONLY"] = "1"
os.environ.setdefault("ROS_LOG_DIR", tempfile.mkdtemp(prefix="withus-mode-only-ros-"))

import rclpy
from ament_index_python.packages import get_package_prefix
from withus_msgs.msg import BodyCommand, DriveCommand, DriverStatus, VehicleStatus
from withus_msgs.srv import SetControl


CAN_FRAME = struct.Struct("=IB3x8s")
COMMAND_IDS = (0x501, 0x502, 0x503, 0x504, 0x506)
FEEDBACK_IDS = (0x10A, 0x301, 0x303, 0x304)
RELEASE_SECONDS = 0.3


def require_vcan(name):
    result = subprocess.run(
        ["ip", "-j", "-details", "link", "show", "dev", name],
        text=True, capture_output=True, timeout=2.0, check=False)
    links = json.loads(result.stdout) if result.returncode == 0 else []
    if len(links) != 1 or links[0].get("linkinfo", {}).get("info_kind") != "vcan":
        raise ValueError("Test requires kernel interface kind vcan: " + name)
    if "UP" not in links[0].get("flags", []):
        raise ValueError("Test requires an already-UP vcan interface: " + name)
    return name


@dataclass(frozen=True)
class Observation:
    at: float
    can_id: int
    length: int
    data: bytes


class Peer:
    """Minimal feedback peer. It never simulates braking or executes motion."""

    def __init__(self, interface):
        self.interface = require_vcan(interface)
        self.lock = threading.Lock()
        self.quit = threading.Event()
        self.thread = None
        self.sock = None
        self.error = None
        self.observations = deque(maxlen=50000)
        self.counters = {can_id: 0 for can_id in FEEDBACK_IDS}
        self.values = {
            "a_raw": 1, "paused": set(), "deny_ad": False,
            "refuse_remote": False, "mode_override": None,
            "gear": 2, "speed": 0.0, "fault": False,
        }
        self.mode = 0

    def start(self):
        self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        try:
            mask = 0xC00007FF
            filters = b"".join(struct.pack("=II", can_id, mask) for can_id in COMMAND_IDS)
            self.sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, filters)
            self.sock.bind((self.interface,))
            self.sock.setblocking(False)
        except BaseException:
            self.sock.close()
            raise
        self.thread = threading.Thread(target=self._run, name="mode_only_peer", daemon=True)
        self.thread.start()
        return self

    def close(self):
        self.quit.set()
        if self.thread:
            self.thread.join(timeout=2.0)
            if self.thread.is_alive():
                raise RuntimeError("Virtual peer did not stop")
        if self.sock:
            self.sock.close()
            self.sock = None

    def inject(self, **values):
        if not set(values).issubset(self.values):
            raise ValueError("Unknown peer injection")
        if "paused" in values:
            values["paused"] = set(values["paused"])
            if not values["paused"].issubset(FEEDBACK_IDS):
                raise ValueError("Unknown paused feedback ID")
        with self.lock:
            self.values.update(values)

    def healthy(self):
        self.inject(a_raw=1, paused=set(), deny_ad=False, refuse_remote=False,
                    mode_override=None, gear=2, speed=0.0, fault=False)
        # This models an independently observed return to manual mode after a
        # fault; it does not claim that loss of CAN automatically does this.
        with self.lock:
            self.mode = 0

    def history(self, since=0.0):
        with self.lock:
            return [frame for frame in self.observations if frame.at >= since]

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
                if can_id == 0x501 and length == 8:
                    request = data[0] & 0x0F
                    if request == 1 and self.values["a_raw"] == 1 and not self.values["deny_ad"]:
                        self.mode = 1
                    elif request == 0 and not self.values["refuse_remote"]:
                        self.mode = 0

    def _feedback(self):
        with self.lock:
            v = self.values
            mode = self.mode if v["mode_override"] is None else v["mode_override"]
            # AD-received stays true even after release: mode 0, not this bit,
            # is the synthetic release acknowledgement in these tests.
            diagnosis = (480 << 22) | (1 << 8) | (int(v["fault"]) << 1)
            status1 = v["gear"] | (mode << 8) | (800 << 16) | (75 << 48)
            raw_speed = round((v["speed"] * 3.6 + 80.0) / 0.1)
            status2 = raw_speed | (350 << 32)
            payloads = {0x10A: v["a_raw"] << 16, 0x301: diagnosis,
                        0x303: status1, 0x304: status2}
            outgoing = []
            for can_id, value in payloads.items():
                if can_id != 0x10A:
                    value |= self.counters[can_id] << 60
                    self.counters[can_id] = (self.counters[can_id] + 1) & 15
                if can_id not in v["paused"]:
                    outgoing.append(CAN_FRAME.pack(can_id, 8, value.to_bytes(8, "little")))
        for packet in outgoing:
            self.sock.send(packet)

    def _run(self):
        next_feedback = time.monotonic()
        try:
            while not self.quit.is_set():
                now = time.monotonic()
                if now >= next_feedback:
                    self._feedback()
                    next_feedback += 0.02
                    if next_feedback < now:
                        next_feedback = now + 0.02
                ready, _, _ = select.select(
                    [self.sock], [], [], max(0.0, min(0.01, next_feedback - time.monotonic())))
                if ready:
                    self._receive()
        except BaseException as error:
            self.error = error
            self.quit.set()


class Harness:
    def __init__(self, executable, peer):
        self.executable = executable
        self.peer = peer
        self.namespace = "/withus_mode_only_" + str(os.getpid())
        self.node = rclpy.create_node("test_client", namespace=self.namespace)
        self.status = None
        self.vehicle = None
        self.process = None
        self.log = None
        self.logs = []
        self.checks = 0
        self.drive = self.node.create_publisher(DriveCommand, "drive_command", 10)
        self.body = self.node.create_publisher(BodyCommand, "body_command", 10)
        self.status_subscription = self.node.create_subscription(
            DriverStatus, "driver_status", self._status, 10)
        self.vehicle_subscription = self.node.create_subscription(
            VehicleStatus, "vehicle_status", self._vehicle, 10)
        self.service = self.node.create_client(SetControl, "set_control")

    def _status(self, msg):
        self.status = msg

    def _vehicle(self, msg):
        self.vehicle = msg

    def check(self, condition, description):
        if not condition:
            reason = self.status.reason if self.status else "no DriverStatus"
            raise AssertionError(description + "; driver reason: " + reason)
        self.checks += 1
        print("PASS " + description, flush=True)

    def step(self):
        if self.peer.error:
            raise RuntimeError("Virtual peer failed: " + str(self.peer.error))
        if self.process and self.process.poll() is not None:
            raise RuntimeError("Driver exited unexpectedly; see " + self.logs[-1])
        rclpy.spin_once(self.node, timeout_sec=0.005)

    def wait(self, predicate, description, timeout=2.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.step()
            if predicate():
                self.check(True, description)
                return
        self.check(False, "Timed out: " + description)

    def pump(self, duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            self.step()

    def start(self, enabled):
        self.status = None
        self.vehicle = None
        fd, path = tempfile.mkstemp(prefix="withus-mode-only-driver-", suffix=".log")
        self.log = os.fdopen(fd, "w")
        self.logs.append(path)
        command = [str(self.executable), "--ros-args", "-r", "__ns:=" + self.namespace,
                   "-p", "can_interface:=" + self.peer.interface,
                   "-p", "enable_tx:=" + str(enabled).lower(),
                   "-p", "control_profile:=hardware_mode_only",
                   "-p", "release_timeout_ms:=300",
                   "-p", "control_request_timeout_ms:=500"]
        self.process = subprocess.Popen(command, stdout=self.log, stderr=subprocess.STDOUT)
        self.wait(lambda: self.status is not None and self.vehicle is not None and
                  self.vehicle.status_301_valid and self.vehicle.status_303_valid and
                  self.vehicle.status_304_valid, "base feedback arrives", 5.0)
        self.wait(self.service.service_is_ready, "SetControl service discovered")
        self.wait(lambda: self.drive.get_subscription_count() > 0 and
                  self.body.get_subscription_count() > 0, "command subscriptions discovered")
        self.check(self.status.control_profile == "hardware_mode_only",
                   "DriverStatus identifies hardware_mode_only profile")
        self.pump(0.1)

    def stop(self, expected_code=0):
        if self.process:
            process = self.process
            self.process = None
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
            try:
                code = process.wait(timeout=4.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2.0)
                raise AssertionError("Owned driver did not shut down within 4 seconds")
            finally:
                if self.log:
                    self.log.close()
                    self.log = None
            self.check(code == expected_code, "driver SIGINT exit code is " + str(expected_code))
        for _ in range(25):
            rclpy.spin_once(self.node, timeout_sec=0.005)

    def close(self):
        if self.process:
            process = self.process
            self.process = None
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=4.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2.0)
        if self.log:
            self.log.close()
        self.node.destroy_node()

    def control(self, enabled, accepted=True):
        request = SetControl.Request()
        request.enable = enabled
        future = self.service.call_async(request)
        self.wait(future.done, "SetControl response received")
        result = future.result()
        self.check(result.accepted == accepted,
                   "SetControl({}) accepted={} ({})".format(enabled, accepted, result.message))
        return result

    def activate(self):
        self.wait(lambda: self.vehicle.status_10a_valid and self.vehicle.remote_a_raw == 1
                  and self.vehicle.drive_mode == 0, "fresh selected A and remote mode observed")
        begin = time.monotonic()
        result = self.control(True)
        self.wait(lambda: self.status.state == DriverStatus.ACTIVE,
                  "new mode 1 acknowledgement enters ACTIVE")
        self.check(result.session_id == self.status.session_id, "active session matches service response")
        self.check(any((f.data[0] & 15) == 1 for f in self.peer.history(begin)),
                   "ACTIVE followed an emitted request 1")

    def only_501(self, since=0.0):
        frames = self.peer.history(since)
        self.check(all(f.can_id == 0x501 for f in frames),
                   "no steering, brake, speed or body CAN frames emitted")
        self.check(all(f.length == 8 and f.data[1:] == bytes(7) and
                       (f.data[0] & 15) in (0, 1) for f in frames),
                   "every emitted 501 has valid DLC, reserved bytes and request flag")

    def assert_release(self, since, destination, description):
        self.wait(lambda: self.status.state == destination, description, 2.0)
        frames = self.peer.history(since)
        zeros = [f for f in frames if f.can_id == 0x501 and (f.data[0] & 15) == 0]
        self.check(len(zeros) >= 8, "release sends repeated request 0")
        self.check(zeros[-1].at - zeros[0].at >= RELEASE_SECONDS - 0.08,
                   "release retains request 0 for the configured burst")
        self.check(zeros[-1].at - zeros[0].at < RELEASE_SECONDS + 0.15,
                   "release burst has a bounded duration")
        self.check(not any((f.data[0] & 15) == 1 for f in frames if f.at > zeros[0].at),
                   "request 1 never resumes after release begins")
        count = len(self.peer.history())
        self.pump(0.16)
        self.check(len(self.peer.history()) == count, "transmission stops after release completion")
        self.only_501(since)

    def recover(self):
        self.peer.healthy()
        self.wait(lambda: self.vehicle.status_10a_valid and self.vehicle.status_301_valid and
                  self.vehicle.status_303_valid and self.vehicle.status_304_valid and
                  self.vehicle.remote_a_raw == 1 and self.vehicle.drive_mode == 0 and
                  self.vehicle.gear == 2 and abs(self.vehicle.speed_mps) < 0.01 and
                  not self.vehicle.motor_fault, "healthy neutral stationary feedback restored")
        self.pump(0.12)
        self.check(self.status.state == DriverStatus.FAULT, "feedback recovery does not clear latched FAULT")
        begin = time.monotonic()
        self.control(True, accepted=False)
        self.control(False)
        self.wait(lambda: self.status.state == DriverStatus.MONITORING,
                  "explicit disable resets fault with fresh remote-mode feedback")
        self.pump(0.12)
        self.check(not self.peer.history(begin), "fault reset does not restart transmission")


def run(h):
    # Disabled transmission stays silent even with all input conditions satisfied.
    begin = time.monotonic()
    h.start(False)
    h.control(True, accepted=False)
    h.pump(0.1)
    h.check(not h.peer.history(begin), "enable_tx=false suppresses all command CAN traffic")
    h.stop()

    # Missing/off/unknown A and stationary/neutral/manual prerequisites.
    h.peer.inject(paused={0x10A})
    h.start(True)
    begin = time.monotonic()
    h.check(h.status.state == DriverStatus.MONITORING, "TX enabled starts in MONITORING")
    h.control(True, accepted=False)
    for raw in (0, 2):
        h.peer.inject(paused=set(), a_raw=raw)
        h.wait(lambda: h.vehicle.status_10a_valid and h.vehicle.remote_a_raw == raw,
               "A raw {} observed".format(raw))
        h.control(True, accepted=False)
    h.peer.inject(a_raw=1, gear=1)
    h.wait(lambda: h.vehicle.remote_a_raw == 1 and h.vehicle.gear == 1, "Drive gear observed")
    h.control(True, accepted=False)
    h.peer.inject(gear=2, speed=0.2)
    h.wait(lambda: h.vehicle.gear == 2 and h.vehicle.speed_mps > 0.1, "nonzero speed observed")
    h.control(True, accepted=False)
    for mode in (1, 3):
        h.peer.inject(speed=0.0, mode_override=mode)
        h.wait(lambda: h.vehicle.drive_mode == mode, "pre-existing mode {} observed".format(mode))
        h.control(True, accepted=False)
    h.check(not h.peer.history(begin), "rejected starts emit no CAN commands")
    h.peer.healthy()

    # Nominal session lasts beyond the drive-command watchdog without commands.
    h.activate()
    begin = time.monotonic()
    h.pump(0.65)
    h.check(h.status.state == DriverStatus.ACTIVE, "ACTIVE persists without DriveCommand keepalives")
    frames = h.peer.history(begin)
    h.check(len(frames) >= 20, "request 1 continues periodically while ACTIVE")
    intervals = [b.at - a.at for a, b in zip(frames, frames[1:])]
    h.check(0.01 < statistics.median(intervals) < 0.035, "median request interval is near 20 ms")
    h.check(all((f.data[0] & 15) == 1 for f in frames), "ACTIVE sends only request 1")
    for cls, publisher, label in ((DriveCommand, h.drive, "DriveCommand"),
                                   (BodyCommand, h.body, "BodyCommand")):
        message = cls()
        message.header.frame_id = "base_link"
        message.header.stamp = h.node.get_clock().now().to_msg()
        message.session_id = h.status.session_id
        if cls is DriveCommand:
            message.gear = 1
            message.speed_mps = 0.2
            message.steering_angle_rad = 0.1
        else:
            message.headlight_on = True
            message.horn_on = True
        before = h.status.rejected_commands
        publisher.publish(message)
        h.wait(lambda: h.status.rejected_commands > before, label + " rejected in mode-only profile")
    h.only_501()
    h.check(h.status.state == DriverStatus.ACTIVE, "rejected movement/body inputs preserve mode-only session")
    begin = time.monotonic()
    h.control(False)
    h.assert_release(begin, DriverStatus.MONITORING, "normal release returns to MONITORING")
    h.check(h.vehicle.ad_signal_received, "mode 0 acknowledges release even while AD-received remains true")

    # Losing a condition requests a bounded release and then latches FAULT.
    failure_cases = [
        ("unknown A value", {"a_raw": 2}),
        ("A deselected", {"a_raw": 0}),
        ("stale A feedback", {"paused": {0x10A}}),
        ("stale diagnosis feedback", {"paused": {0x301}}),
        ("vehicle movement", {"speed": 0.2}),
        ("gear changed to Drive", {"gear": 1}),
        ("motor fault", {"fault": True}),
        ("unexpected drive mode", {"mode_override": 3}),
    ]
    for label, injection in failure_cases:
        h.activate()
        begin = time.monotonic()
        h.peer.inject(**injection)
        h.assert_release(begin, DriverStatus.FAULT, label + " causes release then latched FAULT")
        h.recover()

    # No acquisition ACK: no ACTIVE transition, release zero then a fault.
    h.peer.inject(deny_ad=True)
    begin = time.monotonic()
    h.control(True)
    h.wait(lambda: h.status.state == DriverStatus.REQUESTING, "denied acquisition remains REQUESTING")
    h.assert_release(begin, DriverStatus.FAULT, "request timeout releases and latches FAULT")
    h.recover()

    # A manual-mode ACK that never arrives cannot produce MONITORING.
    h.activate()
    h.peer.inject(refuse_remote=True)
    begin = time.monotonic()
    h.control(False)
    # Repeating false must not prolong the first release's deadline.
    h.pump(0.06)
    h.control(False)
    h.pump(0.06)
    h.control(False)
    h.assert_release(begin, DriverStatus.FAULT, "missing release ACK faults despite repeated disable")
    h.check(time.monotonic() - begin < 0.9, "repeated disable did not extend release deadline")
    h.recover()

    # Counters follow successful 501 transmissions across state/session changes.
    frames = h.peer.history()
    h.check(all((b.data[0] >> 4) == ((a.data[0] >> 4) + 1) % 16
                for a, b in zip(frames, frames[1:])), "501 counter increments modulo 16 without gaps")

    # SIGINT still runs bounded release with the ROS context alive.
    h.activate()
    begin = time.monotonic()
    h.stop()
    frames = h.peer.history(begin)
    zeros = [f for f in frames if (f.data[0] & 15) == 0]
    h.check(len(zeros) >= 8, "SIGINT emits repeated release requests")
    h.check(zeros[-1].at - zeros[0].at >= RELEASE_SECONDS - 0.08,
            "SIGINT preserves the release burst duration")
    h.check(time.monotonic() - begin < 2.0, "SIGINT completes within a bounded interval")
    h.only_501()


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
    peer = Peer(interface).start()
    harness = None
    rclpy.init()
    try:
        harness = Harness(executable, peer)
        run(harness)
        print("ALL {} hardware_mode_only virtual CAN/ROS assertions passed".format(harness.checks), flush=True)
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

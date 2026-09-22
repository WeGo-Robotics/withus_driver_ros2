#!/usr/bin/env python3
"""Verify stationary brake-test commands only on an existing virtual CAN link.

The peer reports synthetic pressure without simulating motion or stopping.
No interface is created, and physical CAN interfaces are rejected before any
socket opens. Only the driver and example children created here are stopped.
"""

import argparse
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time

os.environ.setdefault('ROS_DOMAIN_ID', '182')
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ.setdefault('ROS_LOG_DIR', tempfile.mkdtemp(prefix='withus-brake-ros-'))

import rclpy
from ament_index_python.packages import get_package_prefix
from withus_msgs.msg import BrakeTestCommand, DriverStatus

from test_hardware_mode_only import CAN_FRAME, Observation, require_vcan
from test_hardware_remote_mode import RemoteHarness, RemotePeer


class BrakePeer(RemotePeer):
    """Represent pressure as a test signal, with no vehicle-motion dynamics."""

    def __init__(self, interface):
        super().__init__(interface)
        self.values.update(fault_bits=0, parallel=False)
        self.pressure_raw = 0

    def healthy(self, selected=False):
        super().healthy(selected)
        self.inject(fault_bits=0, parallel=False)

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
                if length != 8:
                    continue
                if can_id == 0x501:
                    request = data[0] & 15
                    if request == 1 and self.values['a_raw'] == 1:
                        self.mode = self.values['ad_mode']
                    elif request == 0 or self.values['a_raw'] == 0:
                        self.mode = 0
                elif can_id == 0x503:
                    # Synthetic response: 20,000 Pa per requested percent.
                    self.pressure_raw = data[1] * 2 if data[0] & 1 else 0

    def _feedback(self):
        with self.lock:
            v = self.values
            mode = self.mode if v['mode_override'] is None else v['mode_override']
            diagnosis = ((480 << 22) | (int(not v['parallel']) << 8) |
                         (int(v['fault']) << 1) | v['fault_bits'])
            status1 = v['gear'] | (mode << 8) | (800 << 16) | (75 << 48)
            speed = round((v['speed'] * 3.6 + 80.0) / 0.1)
            payloads = {0x10A: v['a_raw'] << 16, 0x301: diagnosis,
                        0x303: status1,
                        0x304: speed | (self.pressure_raw << 16) | (350 << 32)}
            packets = []
            for can_id, value in payloads.items():
                if can_id != 0x10A:
                    value |= self.counters[can_id] << 60
                    self.counters[can_id] = (self.counters[can_id] + 1) & 15
                if can_id not in v['paused']:
                    packets.append(CAN_FRAME.pack(can_id, 8, value.to_bytes(8, 'little')))
        for packet in packets:
            self.sock.send(packet)


class BrakeHarness(RemoteHarness):
    def __init__(self, executable, peer):
        super().__init__(executable, peer)
        self.brake = self.node.create_publisher(BrakeTestCommand, 'brake_test_command', 1)
        self.example = None
        self.example_log = None

    def start(self, enabled):
        self.status = None
        self.vehicle = None
        self.states_seen.clear()
        fd, path = tempfile.mkstemp(prefix='withus-brake-driver-', suffix='.log')
        self.log = os.fdopen(fd, 'w')
        self.logs.append(path)
        command = [str(self.executable), '--ros-args', '-r', '__ns:=' + self.namespace,
                   '-p', 'can_interface:=' + self.peer.interface,
                   '-p', 'enable_tx:=' + str(enabled).lower(),
                   '-p', 'control_profile:=hardware_brake_test',
                   '-p', 'brake_test_max_percent:=40.0',
                   '-p', 'brake_test_max_duration_ms:=2000',
                   '-p', 'brake_test_release_duration_ms:=1000']
        self.process = subprocess.Popen(command, stdout=self.log, stderr=subprocess.STDOUT)
        self.wait(lambda: self.status is not None and self.vehicle is not None,
                  'driver status topics arrive', 5.0)
        self.wait(lambda: self.brake.get_subscription_count() > 0,
                  'brake test subscription discovered')
        self.wait(self.service.service_is_ready, 'SetControl service discovered')
        self.check(self.status.control_profile == 'hardware_brake_test',
                   'driver reports the stationary brake-test profile')

    def ready(self):
        self.wait(lambda: self.status.brake_test_ready and not self.status.brake_test_active,
                  'fresh stationary feedback permits a new explicit brake test', 3.0)

    def restore_ready(self):
        # healthy() resets the synthetic mode to 0 before the next 501 restores
        # mode 3. An old ready=true status must not authorize a command across
        # that transition, which correctly retires the previous driver session.
        after = self.node.get_clock().now().nanoseconds
        self.peer.healthy(selected=True)

        def post_reset_feedback():
            for group in ('10a', '301', '303', '304'):
                stamp = getattr(self.vehicle, 'status_' + group + '_stamp')
                if (not getattr(self.vehicle, 'status_' + group + '_valid') or
                        stamp.sec * 1000000000 + stamp.nanosec <= after):
                    return False
            return (self.vehicle.remote_a_raw == 1 and self.vehicle.drive_mode == 3 and
                    self.vehicle.gear == 2 and abs(self.vehicle.speed_mps) < 0.01 and
                    not any(getattr(self.vehicle, field) for field in (
                        'emergency_stop_pressed', 'motor_fault', 'remote_fault', 'dbs_fault',
                        'bms_fault', 'eps_fault', 'front_bumper_triggered',
                        'rear_bumper_triggered', 'parallel_driving_active')))

        self.wait(post_reset_feedback, 'new healthy feedback arrives after the peer reset')
        observed_at = time.monotonic()
        self.wait(lambda: post_reset_feedback() and
                  time.monotonic() - observed_at >= 0.22 and
                  self.status.brake_test_ready and not self.status.brake_test_active and
                  self.status.header.stamp.sec * 1000000000 +
                  self.status.header.stamp.nanosec > after,
                  'post-reset stationary dwell completes before selecting a session', 3.0)

    def message(self, percent, session=None):
        message = BrakeTestCommand()
        message.header.frame_id = 'base_link'
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.session_id = self.status.session_id if session is None else session
        message.brake_percent = float(percent)
        return message

    def rejected(self, message, description):
        before = self.status.rejected_commands
        self.brake.publish(message)
        self.wait(lambda: self.status.rejected_commands > before, description)

    def start_test(self, percent=20.0):
        self.ready()
        session = self.status.session_id
        self.brake.publish(self.message(percent, session))
        self.wait(lambda: self.status.brake_test_active, 'explicit command starts the brake test')
        self.check(self.status.brake_test_ready, 'ready remains true during an admissible active test')
        return session

    def frames(self, since=0.0, can_id=0x503):
        return [f for f in self.peer.history(since) if f.can_id == can_id]

    def release_finished(self, since, session):
        self.wait(lambda: not self.status.brake_test_active and self.status.session_id != session,
                  'test ends and invalidates its previous session', 3.0)
        self.wait(lambda: len([f for f in self.frames(since) if not f.data[0] & 1]) >= 40,
                  'invalid brake commands repeat during the release interval', 2.0)
        self.pump(0.3)
        frames = self.frames(since)
        invalid = [f for f in frames if not f.data[0] & 1]
        self.check(0.9 <= invalid[-1].at - invalid[0].at < 1.2,
                   'the invalid-command phase lasts the configured one second')
        self.check(all(f.data[0] & 1 == 0 and f.data[1:] == bytes(7)
                       for f in frames if f.at >= invalid[0].at),
                   'release clears valid and percent, with no cached valid command replay')
        count = len(self.frames())
        begin = time.monotonic()
        self.pump(0.15)
        self.check(len(self.frames()) == count, '503 transmission stops after invalidation')
        self.heartbeat(begin)

    def run_example(self):
        self.ready()
        fd, path = tempfile.mkstemp(prefix='withus-brake-example-', suffix='.log')
        self.example_log = os.fdopen(fd, 'w')
        self.logs.append(path)
        self.example = subprocess.Popen(
            [sys.executable, '-m', 'withus_examples.brake_response_check',
             '--namespace', self.namespace, '--brake-percent', '12', '--duration', '0.3'],
            stdout=self.example_log, stderr=subprocess.STDOUT)
        begin = time.monotonic()
        self.wait(lambda: self.example.poll() is not None,
                  'Python response-check example finishes on its own', 8.0)
        self.check(self.example.returncode == 0, 'Python example observed its test and inactive feedback')
        self.example_log.close()
        self.example_log = None
        output = Path(path).read_text()
        self.check('Observed brake pressure:' in output and 'max=240000 Pa' in output,
                   'Python example reports the synthetic measured pressure')
        frames = self.frames(begin)
        self.check(any(f.data[0] & 1 and f.data[1] == 12 for f in frames),
                   'Python client requests only the explicitly selected percentage')
        self.check(all(not f.data[0] & 1 or f.data[1] == 12 for f in frames),
                   'Python client never changes the percentage or restarts a new test')

    def close(self):
        if self.example and self.example.poll() is None:
            self.example.terminate()
            try:
                self.example.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self.example.kill()
                self.example.wait(timeout=2.0)
        if self.example_log:
            self.example_log.close()
        super().close()


def run(h):
    begin = time.monotonic()
    h.start(False)
    h.rejected(h.message(20), 'disabled transmission rejects a brake test')
    h.check(not h.peer.history(begin), 'disabled driver emits no command frames')
    h.stop()

    h.start(True)
    begin = time.monotonic()
    h.rejected(h.message(20), 'A-off manual mode cannot start a brake test')
    h.check(not h.frames(), '501 heartbeat alone does not send 503')
    h.peer.inject(a_raw=1)
    h.wait(lambda: h.vehicle.remote_a_raw == 1 and h.vehicle.drive_mode == 3,
           'A selection and semi-automatic mode are observed')
    h.ready()
    h.heartbeat(begin)

    # Every blocking diagnosis remains observable and disallows a brake test.
    faults = [('emergency_stop_pressed', 0), ('motor_fault', 1), ('remote_fault', 5),
              ('dbs_fault', 7), ('bms_fault', 9), ('eps_fault', 10),
              ('front_bumper_triggered', 12), ('rear_bumper_triggered', 13)]
    for name, bit in faults:
        h.peer.inject(fault_bits=1 << bit)
        h.wait(lambda: getattr(h.vehicle, name) and not h.status.brake_test_ready,
               name + ' blocks readiness')
    h.peer.inject(fault_bits=0, parallel=True)
    h.wait(lambda: h.vehicle.parallel_driving_active and not h.status.brake_test_ready,
           'parallel-driving feedback blocks readiness')
    h.peer.inject(parallel=False, gear=1)
    h.wait(lambda: h.vehicle.gear == 1 and not h.status.brake_test_ready, 'Drive gear blocks readiness')
    h.peer.inject(gear=2, speed=0.2)
    h.wait(lambda: h.vehicle.speed_mps > 0.1 and not h.status.brake_test_ready,
           'reported movement blocks readiness')
    h.peer.inject(speed=0.0, a_raw=2)
    h.wait(lambda: h.vehicle.remote_a_raw == 2 and not h.status.brake_test_ready,
           'unknown A value blocks readiness')
    h.restore_ready()
    h.check(not h.frames(), 'readiness and fault changes never start 503 without a command')

    for percent in (-1.0, 41.0, 101.0, math.nan, math.inf):
        h.rejected(h.message(percent), 'invalid or configured-limit percentage is rejected')
    zero_stamp = h.message(20)
    zero_stamp.header.stamp.sec = 0
    zero_stamp.header.stamp.nanosec = 0
    h.rejected(zero_stamp, 'zero timestamp is rejected')
    stale = h.message(20)
    stale.header.stamp.sec -= 1
    h.rejected(stale, 'stale timestamp is rejected')
    future = h.message(20)
    future.header.stamp.sec += 1
    h.rejected(future, 'future timestamp is rejected')
    wrong_frame = h.message(20)
    wrong_frame.header.frame_id = 'map'
    h.rejected(wrong_frame, 'wrong frame_id is rejected')
    h.check(not h.frames(), 'rejected commands do not emit 503')

    begin = time.monotonic()
    session = h.start_test(20)
    h.wait(lambda: h.vehicle.brake_pressure_pa == 400000.0, 'synthetic pressure response arrives')
    h.check(any(f.data[0] & 1 and f.data[1] == 20 for f in h.frames(begin)),
            '503 encodes valid in byte 0 and percentage in byte 1')
    h.release_finished(begin, session)
    h.ready()
    h.rejected(h.message(20, session), 'expired session cannot replay a completed test')

    # Fresh commands cannot extend the first command's two-second hard deadline.
    begin = time.monotonic()
    session = h.start_test(15)
    end = time.monotonic() + 2.3
    while time.monotonic() < end:
        h.brake.publish(h.message(15, session))
        h.pump(0.02)
    h.release_finished(begin, session)
    valid = [f for f in h.frames(begin) if f.data[0] & 1]
    h.check(1.8 < valid[-1].at - valid[0].at < 2.15,
            'fresh keepalives cannot extend the two-second test duration')

    # Losing A, stationarity, or feedback only invalidates the brake command;
    # the continuous 501 request remains independent.
    for label, injection in [('A deselection', {'a_raw': 0}),
                             ('movement', {'speed': 0.2}),
                             ('stale motion feedback', {'paused': {0x304}})]:
        h.restore_ready()
        begin = time.monotonic()
        session = h.start_test(10)
        h.peer.inject(**injection)
        # Keep the command watchdog fresh so loss of feedback, rather than
        # merely stopping this publisher, is what invalidates the command.
        until = time.monotonic() + 0.8
        while h.status.brake_test_active and time.monotonic() < until:
            h.brake.publish(h.message(10, session))
            h.pump(0.02)
        h.release_finished(begin, session)
        h.restore_ready()
        count = len(h.frames())
        h.pump(0.15)
        h.check(len(h.frames()) == count, label + ' recovery does not restart a cached command')
        h.rejected(h.message(10, session), label + ' invalidates the old session')

    h.run_example()
    h.ready()
    h.reject_motion()
    frames = h.peer.history()
    h.check(all(f.can_id in (0x501, 0x503) for f in frames), 'no steering, speed, or body frames are sent')
    h.check(all(f.length == 8 and f.data[2:] == bytes(6) and (f.data[0] & 15) in (0, 1)
                for f in frames), 'all command frames have valid DLC, flags, and reserved bytes')
    for can_id in (0x501, 0x503):
        group = [f for f in frames if f.can_id == can_id]
        h.check(all((b.data[0] >> 4) == ((a.data[0] >> 4) + 1) % 16
                    for a, b in zip(group, group[1:])),
                '{} has an independent modulo-16 counter'.format(hex(can_id)))
    h.check(all(f.data[0] & 15 == 1 for f in frames if f.can_id == 0x501),
            'all brake completion and fault paths preserve the 501 request')

    h.start_test(10)
    begin = time.monotonic()
    h.stop()
    invalid = [f for f in h.frames(begin) if not f.data[0] & 1]
    h.check(len(invalid) >= 40 and invalid[-1].at - invalid[0].at >= 0.9,
            'SIGINT retains the full invalid-brake-command release interval')
    h.check(any(f.data[0] & 15 == 0 for f in h.frames(begin, 0x501)),
            'SIGINT also releases the periodic 501 request')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--interface', default='vcan0')
    parser.add_argument('--driver-prefix', type=Path)
    args = parser.parse_args()
    interface = require_vcan(args.interface)
    prefix = args.driver_prefix or Path(get_package_prefix('withus_driver'))
    executable = prefix / 'lib' / 'withus_driver' / 'withus_driver_node'
    if not executable.is_file():
        raise FileNotFoundError(executable)
    peer = BrakePeer(interface).start()
    harness = None
    rclpy.init()
    try:
        harness = BrakeHarness(executable, peer)
        run(harness)
        print('ALL {} stationary brake CAN/ROS assertions passed'.format(harness.checks), flush=True)
    finally:
        try:
            if harness:
                harness.close()
                print('Driver/example logs: ' + ', '.join(harness.logs), flush=True)
        finally:
            rclpy.shutdown()
            peer.close()


if __name__ == '__main__':
    main()

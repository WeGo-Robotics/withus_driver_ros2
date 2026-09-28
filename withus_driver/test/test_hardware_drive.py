#!/usr/bin/env python3
"""Verify hardware_drive using a synthetic peer on an existing virtual CAN.

The kernel interface must be an already-UP vcan link. This script never creates
or configures a host interface. The peer acknowledges command frames but models
neither vehicle motion nor stopping. Only this test's driver child is stopped.
Source install/setup.bash and use an isolated ROS domain before running.
"""

import argparse
import math
import os
from pathlib import Path
import subprocess
import tempfile
import time

os.environ.setdefault('ROS_DOMAIN_ID', '184')
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ.setdefault('ROS_LOG_DIR', tempfile.mkdtemp(prefix='withus-drive-ros-'))

import rclpy
from ament_index_python.packages import get_package_prefix
from withus_msgs.msg import BodyCommand, DriveCommand, DriverStatus

from can_test_support import CAN_FRAME, FEEDBACK_IDS, CanPeer, Observation, RosHarness, require_vcan


ACTUATORS = (0x502, 0x503, 0x504)
INVALIDATION_SECONDS = 1.0


class DrivePeer(CanPeer):
    """Acknowledge complete drive batches without simulating their dynamics."""

    def __init__(self, interface):
        super().__init__(interface)
        self.values.update(a_raw=0, freeze_gear=False, ad_override=None)
        self.valid_ids = set()
        self.request = 0

    def healthy(self):
        with self.lock:
            self.values.update(a_raw=1, paused=set(), deny_ad=False,
                               refuse_remote=False, mode_override=None,
                               gear=2, speed=0.0, fault=False,
                               freeze_gear=False, ad_override=None)
            self.valid_ids.clear()
            self.mode = 3 if self.request else 0

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
                    self.request = data[0] & 15
                elif can_id in ACTUATORS:
                    if data[0] & 1:
                        self.valid_ids.add(can_id)
                        if can_id == 0x504 and not self.values['freeze_gear']:
                            self.values['gear'] = data[3]
                    else:
                        self.valid_ids.clear()
                if not self.request or self.values['a_raw'] != 1:
                    if not self.values['refuse_remote']:
                        self.mode = 0
                elif self.valid_ids == set(ACTUATORS) and not self.values['deny_ad']:
                    self.mode = 1
                else:
                    self.mode = 3

    def _feedback(self):
        with self.lock:
            v = self.values
            mode = self.mode if v['mode_override'] is None else v['mode_override']
            ad = mode == 1 if v['ad_override'] is None else v['ad_override']
            # AD_State is active-low: bit 6 == 0 means an AD signal was received.
            # Parallel_State is also active-low; bit 8 == 1 means not parallel.
            diagnosis = (480 << 22) | (1 << 8) | (int(not ad) << 6) | (int(v['fault']) << 1)
            status1 = v['gear'] | (mode << 8) | (800 << 16) | (75 << 48)
            raw_speed = round((v['speed'] * 3.6 + 80.0) / 0.1)
            payloads = {0x10A: v['a_raw'] << 16, 0x301: diagnosis,
                        0x303: status1, 0x304: raw_speed | (350 << 32)}
            outgoing = []
            for can_id, value in payloads.items():
                if can_id != 0x10A:
                    value |= self.counters[can_id] << 60
                    self.counters[can_id] = (self.counters[can_id] + 1) & 15
                if can_id not in v['paused']:
                    outgoing.append(CAN_FRAME.pack(can_id, 8, value.to_bytes(8, 'little')))
        for packet in outgoing:
            self.sock.send(packet)


class DriveHarness(RosHarness):
    def check(self, condition, description):
        if not condition and self.status:
            description += '; drive reason: ' + self.status.drive_reason
        super().check(condition, description)

    def start(self, enabled):
        self.status = None
        self.vehicle = None
        fd, path = tempfile.mkstemp(prefix='withus-drive-driver-', suffix='.log')
        self.log = os.fdopen(fd, 'w')
        self.logs.append(path)
        command = [str(self.executable), '--ros-args', '-r', '__ns:=' + self.namespace,
                   '-p', 'can_interface:=' + self.peer.interface,
                   '-p', 'enable_tx:=' + str(enabled).lower(),
                   '-p', 'control_profile:=hardware_drive',
                   '-p', 'command_timeout_ms:=200',
                   '-p', 'control_request_timeout_ms:=1000',
                   '-p', 'drive_invalidation_duration_ms:=1000',
                   '-p', 'release_timeout_ms:=1000',
                   '-p', 'stop_confirm_duration_ms:=200']
        self.process = subprocess.Popen(command, stdout=self.log, stderr=subprocess.STDOUT)
        self.wait(lambda: self.status is not None and self.vehicle is not None,
                  'driver status topics arrive', 5.0)
        self.wait(self.service.service_is_ready, 'SetControl service discovered')
        self.wait(lambda: self.drive.get_subscription_count() > 0 and
                  self.body.get_subscription_count() > 0, 'command subscriptions discovered')
        self.check(self.status.control_profile == 'hardware_drive', 'hardware_drive profile reported')

    def message(self, gear=2, speed=0.0, steering=0.1, brake=17.0, session=None):
        message = DriveCommand()
        message.header.frame_id = 'base_link'
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.session_id = self.status.session_id if session is None else session
        message.gear = gear
        message.speed_mps = float(speed)
        message.steering_angle_rad = float(steering)
        message.brake_percent = float(brake)
        return message

    def rejected(self, message, description, publisher=None):
        before = self.status.rejected_commands
        (publisher or self.drive).publish(message)
        self.wait(lambda: self.status.rejected_commands > before, description)

    def stream(self, duration, predicate=None, description=None, session=None, **values):
        # A stream belongs to one explicitly selected session. Never silently
        # adopt a replacement session and accidentally restart after a fault.
        session = self.status.session_id if session is None else session
        next_publish = 0.0
        end = time.monotonic() + duration
        while time.monotonic() < end:
            if time.monotonic() >= next_publish:
                self.drive.publish(self.message(session=session, **values))
                next_publish = time.monotonic() + 0.04
            self.step()
            if predicate and predicate():
                self.check(True, description)
                return
        if predicate:
            self.check(False, 'Timed out: ' + description)

    def frames(self, since=0.0, can_id=None):
        return [f for f in self.peer.history(since)
                if (f.can_id in ACTUATORS if can_id is None else f.can_id == can_id)]

    def heartbeat(self, since):
        frames = self.frames(since, 0x501)
        self.check(len(frames) >= 3 and all(f.data[0] & 15 == 1 for f in frames),
                   '501 request 1 continues independently of drive command state')
        self.check(self.status.state == DriverStatus.MONITORING and self.status.control_requested,
                   'main state describes continuous 501, not drive ownership')

    def restore(self):
        before = self.node.get_clock().now().nanoseconds
        self.peer.healthy()

        def fresh_reset():
            if self.vehicle is None:
                return False
            for group in ('10a', '301', '303', '304'):
                stamp = getattr(self.vehicle, 'status_' + group + '_stamp')
                if (not getattr(self.vehicle, 'status_' + group + '_valid') or
                        stamp.sec * 1000000000 + stamp.nanosec <= before):
                    return False
            return (self.vehicle.drive_mode == 3 and self.vehicle.gear == 2 and
                    self.vehicle.remote_a_raw == 1 and not self.vehicle.motor_fault and
                    abs(self.vehicle.speed_mps) < 0.01)

        self.wait(fresh_reset, 'new healthy mode-3 feedback arrives after peer reset')
        self.wait(lambda: fresh_reset() and self.status.drive_ready and
                  not self.status.drive_active, 'stationary dwell permits a new explicit command', 3.0)

    def bootstrap(self, brake=17.0):
        self.wait(lambda: self.status.drive_ready, 'drive command admission ready')
        session = self.status.session_id
        self.stream(1.0, lambda: self.status.drive_active and self.status.drive_mode_confirmed and
                    self.vehicle.drive_mode == 1 and self.vehicle.ad_signal_received,
                    'explicit zero command plus new mode-1 feedback confirms drive mode',
                    session=session, brake=brake)
        self.check(self.vehicle.drive_mode == 1 and self.vehicle.ad_signal_received,
                   'vehicle reports mode 1 and received AD signal without relabeling mode 3')
        return session

    def invalidated(self, since, session):
        self.wait(lambda: not self.status.drive_active and self.status.session_id != session,
                  'old drive session is retired', 3.0)
        self.wait(lambda: all(any(not f.data[0] & 1 for f in self.frames(since, can_id))
                              for can_id in ACTUATORS),
                  'all three actuator valid bits are cleared')
        invalid_start = min(f.at for f in self.frames(since) if not f.data[0] & 1)
        self.pump(max(0.0, invalid_start + INVALIDATION_SECONDS + 0.15 - time.monotonic()))
        for can_id in ACTUATORS:
            frames = self.frames(invalid_start, can_id)
            self.check(len(frames) >= 40 and all(not f.data[0] & 1 for f in frames),
                       '0x{:03X} repeats invalid commands without replay'.format(can_id))
            self.check(0.85 <= frames[-1].at - frames[0].at < 1.2,
                       '0x{:03X} invalidation is bounded to one second'.format(can_id))
        self.check(all(f.data[1] == 0 for f in self.frames(invalid_start, 0x503)),
                   'invalidation does not invent a 25% or 100% braking policy')
        self.check(all(int.from_bytes(f.data[6:8], 'little') == 0
                       for f in self.frames(invalid_start, 0x504)),
                   'invalid speed frames contain a zero target')
        count = len(self.frames())
        begin = time.monotonic()
        self.pump(0.15)
        self.check(len(self.frames()) == count, 'actuator transmission stops after bounded invalidation')
        self.heartbeat(begin)


def run(h):
    begin = time.monotonic()
    h.start(False)
    h.rejected(h.message(), 'enable_tx=false rejects drive commands')
    h.pump(0.1)
    h.check(not h.peer.history(begin), 'disabled driver emits no CAN commands')
    h.stop()

    h.peer.inject(paused=set(FEEDBACK_IDS))
    h.start(True)
    counter_start = time.monotonic()
    h.pump(0.3)
    h.heartbeat(counter_start)
    h.check(not h.frames(counter_start) and not h.status.drive_ready,
            'startup without feedback sends no actuator frames')
    h.rejected(h.message(), 'missing feedback rejects even a zero command')
    h.peer.inject(paused=set(), a_raw=0)
    h.wait(lambda: h.vehicle.status_10a_valid and h.vehicle.remote_a_raw == 0,
           'A-off feedback arrives')
    h.rejected(h.message(), 'A-off rejects drive commands while heartbeat continues')
    h.restore()
    h.check(not h.status.drive_mode_confirmed, 'mode 3 alone is not drive-mode confirmation')
    h.rejected(h.message(gear=1, speed=0.2, brake=0), 'mode-3 startup rejects a nonzero target')
    h.rejected(h.message(gear=1), 'first zero command must preserve observed gear')
    h.check(not h.frames(counter_start), 'no drive frames precede the first accepted explicit command')

    for label, values in (
            ('NaN steering', {'steering': math.nan}),
            ('infinite speed', {'speed': math.inf}),
            ('oversized steering', {'steering': 0.5}),
            ('oversized brake', {'brake': 101}),
            ('negative speed', {'speed': -0.1})):
        h.rejected(h.message(**values), label + ' is rejected')
    stale = h.message()
    stale.header.stamp.sec -= 2
    h.rejected(stale, 'stale ROS timestamp rejected')
    future = h.message()
    future.header.stamp.sec += 2
    h.rejected(future, 'future ROS timestamp rejected')
    h.rejected(h.message(session=h.status.session_id + 1), 'wrong session rejected')
    body = BodyCommand()
    body.header.frame_id = 'base_link'
    body.header.stamp = h.node.get_clock().now().to_msg()
    body.session_id = h.status.session_id
    body.horn_on = True
    h.rejected(body, 'hardware_drive rejects BodyCommand', h.body)

    begin = time.monotonic()
    session = h.bootstrap()
    h.stream(0.2, session=session)
    for can_id in ACTUATORS:
        h.check(any(f.data[0] & 1 for f in h.frames(begin, can_id)),
                'explicit command enables 0x{:03X}'.format(can_id))
    h.check(all(f.data[1] == 17 for f in h.frames(begin, 0x503)),
            'client-selected brake percentage is passed through unchanged')
    steering_raw = round((math.degrees(0.1) + 30.0) / 0.1)
    h.check(all(int.from_bytes(f.data[4:6], 'little') == steering_raw
                for f in h.frames(begin, 0x502)), 'client steering uses the documented encoding')

    # A gear transition starts at zero, waits for post-send feedback, and never
    # caches a prematurely submitted positive-speed target.
    h.peer.inject(freeze_gear=True)
    gear_begin = time.monotonic()
    h.stream(0.5, lambda: h.status.drive_gear_pending,
             'zero-speed gear request waits for new gear feedback', session=session, gear=1)
    h.rejected(h.message(gear=1, speed=0.4, brake=0, session=session),
               'positive speed rejected while gear acknowledgement is pending')
    h.stream(0.15, session=session, gear=1)
    h.check(all(int.from_bytes(f.data[6:8], 'little') == 0
                for f in h.frames(gear_begin, 0x504)), 'speed stays zero throughout gear acquisition')
    queued_before_ack = h.message(gear=1, speed=0.4, brake=0, session=session)
    h.peer.inject(freeze_gear=False)
    # Do not publish another zero update here: that would make the queued
    # timestamp fail the ordinary last-accepted-stamp test as well. The timer
    # keeps transmitting the already accepted zero-speed gear request.
    h.wait(lambda: not h.status.drive_gear_pending and h.vehicle.gear == 1,
           'new Drive feedback acknowledges the sent gear request', 0.15)
    queued_stamp = queued_before_ack.header.stamp
    queued_age = h.node.get_clock().now().nanoseconds - (
        queued_stamp.sec * 1000000000 + queued_stamp.nanosec)
    h.check(0 <= queued_age < 150000000,
            'queued positive command is still within its ordinary timestamp age limit')
    h.rejected(queued_before_ack,
               'pre-ack positive command delivered after gear confirmation is rejected')
    h.stream(0.12, session=session, gear=1)
    h.check(all(int.from_bytes(f.data[6:8], 'little') == 0
                for f in h.frames(gear_begin, 0x504)), 'gear acknowledgement never replays rejected speed')
    h.rejected(h.message(gear=1, speed=0.4, brake=7, session=session),
               'simultaneous nonzero speed and brake requests are rejected')
    positive_begin = time.monotonic()
    h.stream(0.2, session=session, gear=1, speed=0.4, brake=0)
    h.check(any(f.data[0] & 1 and f.data[3] == 1 and
                int.from_bytes(f.data[6:8], 'little') == round(0.4 * 36)
                for f in h.frames(positive_begin, 0x504)), 'new post-ack command sends requested Drive speed')
    braking_begin = time.monotonic()
    h.stream(0.2, session=session, gear=1, speed=0.0, brake=7)
    h.check(any(f.data[0] & 1 and f.data[1] == 7 for f in h.frames(braking_begin, 0x503)),
            'ordinary stopping passes through the client-selected 7% brake request')
    h.check(all(f.data[1] in (17, 0, 7) for f in h.frames(begin, 0x503)),
            'ordinary driving uses only explicit client braking values')

    # Watchdog and A/fault/mode/freshness losses all retire the session. No
    # service call is required for a fresh, stationary, explicit restart.
    expired_at = time.monotonic()
    h.invalidated(expired_at, session)
    h.restore()
    h.rejected(h.message(session=session), 'expired session cannot restart drive output')
    for label, injection in (
            ('A released', {'a_raw': 0}),
            ('unknown A', {'a_raw': 2}),
            ('motor fault', {'fault': True}),
            ('mode reverted to 3', {'mode_override': 3}),
            ('unexpected mode 15', {'mode_override': 15}),
            ('AD signal lost', {'ad_override': False}),
            ('unexpected gear', {'gear': 3, 'freeze_gear': True}),
            ('stale 10A', {'paused': {0x10A}}),
            ('stale 301', {'paused': {0x301}}),
            ('stale 303', {'paused': {0x303}}),
            ('stale 304', {'paused': {0x304}})):
        h.restore()
        session = h.bootstrap()
        failure_at = time.monotonic()
        h.peer.inject(**injection)
        # Keep the old session's command fresh so stale-feedback cases cannot
        # accidentally pass by exercising only the 200 ms command watchdog.
        h.stream(0.6, lambda: h.status.session_id != session,
                 label + ' retires the old session despite a live client', session=session)
        h.invalidated(failure_at, session)

    # Mode and gear timeouts require continued valid command publication.
    h.restore()
    h.peer.inject(deny_ad=True)
    session = h.status.session_id
    begin = time.monotonic()
    h.stream(1.8, lambda: h.status.session_id != session,
             'missing mode-1 acknowledgement has a bounded request timeout', session=session)
    h.check(not h.status.drive_mode_confirmed, 'mode timeout never claims drive confirmation')
    h.invalidated(begin, session)
    h.restore()
    session = h.bootstrap()
    h.peer.inject(freeze_gear=True)
    begin = time.monotonic()
    h.stream(1.8, lambda: h.status.session_id != session,
             'missing gear acknowledgement times out with a live client', session=session, gear=1)
    h.invalidated(begin, session)

    # Optional disable and process shutdown invalidate actuators as well as
    # sending a bounded 501 release. Resume never restores a cached target.
    h.restore()
    session = h.bootstrap()
    begin = time.monotonic()
    h.control(False)
    h.wait(lambda: not h.status.drive_active and h.status.session_id != session,
           'optional disable retires the drive session')
    h.wait(lambda: not h.status.control_requested and h.status.state == DriverStatus.MONITORING,
           'optional disable completes its 501 release', 2.0)
    h.pump(0.2)
    h.check(all(any(not f.data[0] & 1 for f in h.frames(begin, can_id)) for can_id in ACTUATORS),
            'optional disable invalidates steering, brake, and speed')
    count = len(h.peer.history())
    h.pump(0.15)
    h.check(len(h.peer.history()) == count, 'pause eventually silences all command IDs')
    h.control(True)
    begin = time.monotonic()
    h.pump(0.25)
    h.heartbeat(begin)
    h.check(not h.frames(begin), 'resume does not replay the previous drive target')
    h.restore()
    h.rejected(h.message(session=session), 'pre-pause session is rejected after resume')
    h.bootstrap()
    begin = time.monotonic()
    h.stop()
    for can_id in ACTUATORS:
        invalid = [f for f in h.frames(begin, can_id) if not f.data[0] & 1]
        h.check(len(invalid) >= 40 and invalid[-1].at - invalid[0].at >= 0.85,
                'SIGINT preserves bounded invalidation for 0x{:03X}'.format(can_id))
    zeros = [f for f in h.frames(begin, 0x501) if f.data[0] & 15 == 0]
    h.check(len(zeros) >= 40 and zeros[-1].at - zeros[0].at >= 0.85,
            'SIGINT also sends the repeated 501 request-zero release')
    h.check(time.monotonic() - begin < 4.0, 'SIGINT shutdown is bounded')
    frames = h.peer.history(counter_start)
    h.check(all(f.can_id in (0x501,) + ACTUATORS and f.length == 8 for f in frames),
            'only DLC-8 mode/steering/brake/speed frames are sent; no body frame')
    for can_id in (0x501,) + ACTUATORS:
        sequence = [f for f in frames if f.can_id == can_id]
        h.check(all((b.data[0] >> 4) == ((a.data[0] >> 4) + 1) % 16
                    for a, b in zip(sequence, sequence[1:])),
                '0x{:03X} counter independently advances modulo 16'.format(can_id))


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
    peer = DrivePeer(interface).start()
    harness = None
    rclpy.init()
    try:
        harness = DriveHarness(executable, peer)
        run(harness)
        print('ALL {} hardware_drive virtual CAN/ROS assertions passed'.format(harness.checks), flush=True)
    finally:
        try:
            if harness:
                harness.close()
                print('Driver logs: ' + ', '.join(harness.logs), flush=True)
        finally:
            rclpy.shutdown()
            peer.close()


if __name__ == '__main__':
    main()

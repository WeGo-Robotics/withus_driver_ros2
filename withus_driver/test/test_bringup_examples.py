#!/usr/bin/env python3
"""Verify separate installed driver, status, and keyboard launches on isolated vcan.

Run inside an isolated user/network namespace with an already-UP virtual CAN.
The synthetic peer acknowledges targets but models no motion or stopping.
Only test-owned processes and pseudo-terminals are opened or stopped.
"""

import argparse
import errno
import os
from pathlib import Path
import pty
import signal
import subprocess
import tempfile
import termios
import time

os.environ.setdefault('ROS_DOMAIN_ID', '186')
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ.setdefault('ROS_LOG_DIR', tempfile.mkdtemp(prefix='withus-launch-test-'))

import rclpy
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from withus_msgs.msg import DriverStatus, VehicleStatus

from can_test_support import require_vcan
from test_hardware_drive import ACTUATORS, DrivePeer


class TerminalLaunch:
    """Give one launch its own foreground controlling terminal, without a GUI."""

    def __init__(self, command, log):
        self.master, self.slave = pty.openpty()
        self.original = termios.tcgetattr(self.slave)
        os.set_blocking(self.master, False)
        # A fresh interpreter owns the terminal before exec. Avoid preexec_fn
        # because the CAN peer already has a thread in this test process.
        wrapper = ('import fcntl, os, sys, termios; os.setsid(); '
                   'fcntl.ioctl(0, termios.TIOCSCTTY, 0); '
                   'os.execvp(sys.argv[1], sys.argv[1:])')
        self.process = subprocess.Popen(
            ['python3', '-c', wrapper, *command], stdin=self.slave,
            stdout=log, stderr=subprocess.STDOUT)
        self.output = bytearray()

    def read(self):
        while True:
            try:
                chunk = os.read(self.master, 4096)
            except BlockingIOError:
                return
            except OSError as error:
                if error.errno == errno.EIO:
                    return
                raise
            if not chunk:
                return
            self.output.extend(chunk)

    def send(self, keys):
        os.write(self.master, keys.encode() if isinstance(keys, str) else keys)

    def close(self):
        os.close(self.master)
        os.close(self.slave)


class LaunchCheck:
    def __init__(self, interface):
        self.peer = DrivePeer(interface).start()
        self.peer.healthy()
        self.node = rclpy.create_node('launch_check')
        self.status = None
        self.vehicle = None
        self.sub = self.node.create_subscription(
            DriverStatus, '/withus/driver_status', self.on_status, 10)
        self.vehicle_sub = self.node.create_subscription(
            VehicleStatus, '/withus/vehicle_status', self.on_vehicle, 10)
        self.processes = []
        self.terminals = []
        self.checks = 0

    def on_status(self, message):
        self.status = message

    def on_vehicle(self, message):
        self.vehicle = message

    def check(self, condition, description):
        if not condition:
            reason = self.status.drive_reason if self.status else 'no DriverStatus'
            raise AssertionError(description + '; driver: ' + reason)
        self.checks += 1
        print('PASS ' + description, flush=True)

    def step(self):
        if self.peer.error:
            raise RuntimeError(str(self.peer.error))
        for terminal in self.terminals:
            terminal.read()
        rclpy.spin_once(self.node, timeout_sec=0.01)

    def pump(self, duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            self.step()

    def wait(self, predicate, description, timeout=6.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.step()
            if predicate():
                self.check(True, description)
                return
        self.check(False, 'Timed out: ' + description)

    def start(self, package, launch, *arguments, terminal=False):
        fd, path = tempfile.mkstemp(prefix='withus-launch-', suffix='.log')
        log = os.fdopen(fd, 'w')
        command = ['ros2', 'launch', package, launch, *arguments]
        if terminal:
            tty = TerminalLaunch(command, log)
            self.terminals.append(tty)
            process = tty.process
            tty.log_path = Path(path)
        else:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                       start_new_session=True)
        self.processes.append((process, log, Path(path)))
        return (tty if terminal else process), Path(path)

    def stop(self, process):
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
        self.wait(lambda: process.poll() is not None, 'owned launch exits', 6.0)
        self.check(process.returncode == 0, 'owned launch exits cleanly')

    def frames(self, since=0.0, can_id=None, valid=True):
        return [frame for frame in self.peer.history(since)
                if (frame.can_id in ACTUATORS if can_id is None else frame.can_id == can_id)
                and (valid is None or bool(frame.data[0] & 1) == valid)]

    def target(self, since, gear=None, speed=None, brake=None, steering=None):
        """Observe all requested target components in recent valid CAN frames."""
        latest = {frame.can_id: frame for frame in self.frames(since)}
        if not set(ACTUATORS).issubset(latest):
            return False
        accelerate = latest[0x504].data
        return ((gear is None or accelerate[3] == gear) and
                (speed is None or int.from_bytes(accelerate[6:8], 'little') == round(speed * 36)) and
                (brake is None or latest[0x503].data[1] == brake) and
                (steering is None or int.from_bytes(latest[0x502].data[4:6], 'little') ==
                 round((steering * 180 / 3.141592653589793 + 30) / 0.1)))

    def key_target(self, tty, key, description, **target):
        since = time.monotonic()
        tty.send(key)
        self.wait(lambda: self.target(since, **target), description, 2.0)
        return since

    def shift(self, tty, key, gear):
        log_offset = tty.log_path.stat().st_size
        since = self.key_target(tty, key, 'key ' + key + ' requests zero-speed gear ' + str(gear),
                                gear=gear, speed=0.0)
        self.wait(lambda: self.vehicle.gear == gear and self.status.drive_mode_confirmed and
                  not self.status.drive_gear_pending,
                  'fresh gear feedback acknowledges key ' + key)
        name = ('P', 'D', 'N', 'R')[gear]
        self.wait(lambda: name + ' confirmed. Press a new' in
                  tty.log_path.read_text()[log_offset:],
                  'keyboard client consumes the new ' + name + ' acknowledgement')
        self.check(all(int.from_bytes(f.data[6:8], 'little') == 0
                       for f in self.frames(since, 0x504)),
                   'gear transition never includes a cached positive speed')

    def close(self):
        for process, log, path in reversed(self.processes):
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=6.0)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=2.0)
            log.close()
            print('Launch log: ' + str(path), flush=True)
        for terminal in self.terminals:
            terminal.close()
        self.node.destroy_node()
        self.peer.close()


def run(h, interface):
    bringup_share = Path(get_package_share_directory('withus_bringup'))
    h.check({p.name for p in (bringup_share / 'launch').glob('*.launch.py')} ==
            {'withus_bringup.launch.py'}, 'only the standard driver launch is installed')
    example_share = Path(get_package_share_directory('withus_examples'))
    h.check({p.name for p in (example_share / 'launch').glob('*.launch.py')} ==
            {'status_monitor.launch.py', 'keyboard_teleop.launch.py'},
            'only status and keyboard client launches are installed')
    h.check(not (Path(get_package_share_directory('withus_msgs')) /
                 'msg' / 'BrakeTestCommand.msg').exists(), 'obsolete brake interface is not installed')

    executable = Path(get_package_prefix('withus_driver')) / 'lib/withus_driver/withus_driver_node'
    for profile in ('monitor_only', 'hardware_mode_only', 'hardware_remote_mode', 'hardware_brake_test'):
        result = subprocess.run([str(executable), '--ros-args', '-p', 'control_profile:=' + profile,
                                 '-p', 'can_interface:=' + interface],
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=3.0)
        h.check(result.returncode != 0 and 'control_profile must be' in result.stdout,
                'removed profile rejected: ' + profile)

    driver, _ = h.start('withus_bringup', 'withus_bringup.launch.py', 'can_interface:=' + interface)
    h.wait(lambda: h.status is not None and h.vehicle is not None and h.status.drive_ready,
           'default bringup becomes ready')
    h.check(h.status.control_profile == 'hardware_drive' and h.status.enable_tx and
            h.status.control_requested, 'default profile and TX are hardware_drive and enabled')
    h.check(not h.status.drive_active and {f.can_id for f in h.peer.history()} == {0x501},
            'driver startup sends only 501 until a client command arrives')

    monitor, monitor_log = h.start('withus_examples', 'status_monitor.launch.py',
                                   'report_period_sec:=0.1')
    h.wait(lambda: 'ROS status reception active.' in monitor_log.read_text(),
           'status client receives the separately running driver')
    tty, keyboard_log = h.start('withus_examples', 'keyboard_teleop.launch.py', terminal=True)
    h.wait(lambda: not (termios.tcgetattr(tty.slave)[3] & termios.ICANON),
           'keyboard launch reads its own foreground terminal without a GUI')
    h.wait(lambda: 'Driver limits:' in keyboard_log.read_text(),
           'keyboard fetches configured limits from the separate driver')
    h.pump(0.3)
    h.check(tty.process.poll() is None and monitor.poll() is None,
            'keyboard and status clients run simultaneously')
    h.check(sum(name == 'withus_driver' and namespace == '/withus'
                for name, namespace in h.node.get_node_names_and_namespaces()) == 1,
            'client launches do not start another driver')
    h.check({f.can_id for f in h.peer.history()} == {0x501},
            'starting both clients adds no actuator commands before a key')

    h.key_target(tty, 's', 's bootstraps with current gear, 80 percent brake and centered steering',
                 gear=2, speed=0.0, brake=80, steering=0.0)
    h.wait(lambda: h.status.drive_active and h.status.drive_mode_confirmed,
           'fresh full-AD feedback confirms the initial stationary command')
    h.wait(lambda: 'AD confirmed. Press w/a/d' in keyboard_log.read_text(),
           'keyboard observes full AD before accepting a gear key')
    h.shift(tty, '2', 1)
    h.key_target(tty, 'w', 'fresh w after D acknowledgement commands the initial 0.1 m/s',
                 gear=1, speed=0.1, brake=0, steering=0.0)
    h.key_target(tty, 'q', 'q raises speed by 0.1 m/s', speed=0.2, brake=0)
    h.key_target(tty, 'z', 'z lowers speed by 0.1 m/s', speed=0.1, brake=0)
    h.key_target(tty, 'a', 'a uses the configured positive steering limit', speed=0.1, steering=0.3)
    h.key_target(tty, 'd', 'd uses the configured negative steering limit', speed=0.1, steering=-0.3)
    h.key_target(tty, 's', 's commands zero speed, 80 percent brake and steering neutral',
                 speed=0.0, brake=80, steering=0.0)

    # Arrow escape sequences must not become their alphabetic suffix commands.
    since = time.monotonic()
    tty.send(b'\x1b[A\x1b[B\x1b[C\x1b[D')
    h.pump(0.15)
    h.check(h.target(since, speed=0.0, brake=80, steering=0.0),
            'arrow sequences do not trigger movement or steering keys')

    h.key_target(tty, 'w', 'w releases the client brake for a fresh motion request', speed=0.1, brake=0)
    since = time.monotonic()
    timed_out_session = h.status.session_id
    h.wait(lambda: h.status.session_id != timed_out_session and not h.status.drive_active,
           'one second without input ends publishing and expires the drive session', 3.0)
    h.wait(lambda: h.status.drive_ready and not h.status.drive_active,
           'command watchdog completes bounded invalidation after keyboard input timeout')
    h.check(all(f.data[1] == 0 for f in h.frames(since, 0x503)),
            'input timeout never inserts an automatic 80 percent brake command')
    h.check(all(any(f.can_id == can_id for f in h.frames(since, valid=False))
                for can_id in ACTUATORS),
            'driver watchdog invalidates all three actuators after client publication ends')
    h.pump(0.15)
    h.check(not h.frames(time.monotonic() - 0.1),
            'keyboard input timeout never automatically rearms or replays motion')
    h.key_target(tty, 's', 'explicit s rearms after idle publication ended',
                 gear=1, speed=0.0, brake=80, steering=0.0)
    h.wait(lambda: h.status.drive_mode_confirmed, 'rearmed session confirms AD again')
    h.pump(0.06)

    # The peer reports motion explicitly; it never pretends commanded motion is real.
    h.peer.inject(speed=0.2)
    h.wait(lambda: h.vehicle.speed_mps > 0.1, 'synthetic moving feedback reaches the client')
    since = time.monotonic()
    tty.send('4')
    h.pump(0.25)
    h.check(h.frames(since, 0x504) and all(f.data[3] == 1 for f in h.frames(since, 0x504)),
            'gear key while moving does not request a Reverse shift')
    h.peer.inject(speed=0.0)
    h.wait(lambda: abs(h.vehicle.speed_mps) < 0.01, 'synthetic stopped feedback arrives')
    h.pump(0.3)
    for key, gear in [('1', 0), ('3', 2), ('4', 3)]:
        h.shift(tty, key, gear)
    h.key_target(tty, 'w', 'Reverse uses a nonnegative 0.1 m/s speed magnitude', gear=3, speed=0.1, brake=0)

    # Speed changes respect the driver-provided reverse limit rather than exceeding it.
    for _ in range(12):
        tty.send('q')
        h.pump(0.025)
    h.wait(lambda: h.target(time.monotonic() - 0.1, gear=3, speed=0.5),
           'repeated q is capped at the driver reverse-speed limit')
    since = time.monotonic()
    old_session = h.status.session_id
    h.peer.inject(a_raw=0)
    h.wait(lambda: h.status.session_id != old_session and not h.status.drive_active,
           'A-off retires the keyboard driving session')
    h.peer.inject(a_raw=1)
    h.wait(lambda: h.status.drive_ready and not h.status.drive_active,
           'A reselection becomes ready without automatically resuming the client')
    h.pump(0.2)
    h.check(not h.frames(time.monotonic() - 0.15),
            'A cycle never replays the previous speed target')
    h.check(all(f.data[0] & 15 == 1 for f in h.peer.history(since) if f.can_id == 0x501),
            '501 request remains continuous during A-off and command invalidation')
    tty.send('w')
    h.pump(0.15)
    h.check(not h.status.drive_active, 'motion key cannot silently rearm a retired session')
    h.key_target(tty, 's', 'explicit s rearms the new session with a stationary brake target',
                 gear=3, speed=0.0, brake=80, steering=0.0)
    h.wait(lambda: h.status.drive_mode_confirmed, 'new session again confirms full AD feedback')

    h.pump(0.06)
    h.key_target(tty, 'w', 'fresh w resumes only after explicit rearm and mode confirmation',
                 gear=3, speed=0.5, brake=0, steering=0.0)
    since = time.monotonic()
    tty.send(b'\x03')
    h.wait(lambda: tty.process.poll() is not None, 'Ctrl+C ends keyboard command publication', 5.0)
    h.check(tty.process.returncode == 0, 'keyboard launch exits cleanly after Ctrl+C')
    h.check(termios.tcgetattr(tty.slave) == tty.original, 'Ctrl+C restores original terminal settings')
    h.check(all(f.data[1] == 0 for f in h.frames(since, 0x503)),
            'Ctrl+C never inserts an automatic brake command')
    h.wait(lambda: h.status.drive_ready and not h.status.drive_active,
           'driver returns to ready after client command expiry and invalidation')
    h.check(driver.poll() is None and monitor.poll() is None and h.status.control_requested,
            'keyboard exit leaves the driver, status monitor and 501 running')
    h.stop(monitor)
    h.check(driver.poll() is None, 'stopping the status client also leaves the driver running')
    h.stop(driver)
    print('Keyboard launch log: ' + str(keyboard_log), flush=True)

    # Opting out of TX still works without a third profile.
    h.status = None
    since_ros = h.node.get_clock().now().nanoseconds
    since_can = time.monotonic()
    receiver, _ = h.start('withus_bringup', 'withus_bringup.launch.py',
                          'can_interface:=' + interface, 'enable_tx:=false')
    h.wait(lambda: h.status is not None and
           h.status.header.stamp.sec * 1000000000 + h.status.header.stamp.nanosec > since_ros and
           h.status.feedback_valid and not h.status.enable_tx,
           'enable_tx=false receives feedback using hardware_drive')
    h.check(not h.peer.history(since_can), 'TX opt-out sends no CAN command frames')
    h.stop(receiver)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--interface', default='vcan0')
    args = parser.parse_args()
    interface = require_vcan(args.interface)
    rclpy.init()
    harness = None
    try:
        harness = LaunchCheck(interface)
        run(harness, interface)
        print('ALL {} installed driver/client launch assertions passed'.format(harness.checks), flush=True)
    finally:
        if harness:
            harness.close()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

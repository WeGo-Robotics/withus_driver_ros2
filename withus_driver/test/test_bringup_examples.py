#!/usr/bin/env python3
"""Verify separate installed driver, status, and keyboard launches on isolated vcan.

Run inside an isolated user/network namespace with an already-UP virtual CAN.
The synthetic peer acknowledges targets but models no motion or stopping.
Only test-owned processes and pseudo-terminals are opened or stopped.
"""

import argparse
import errno
import fcntl
import os
from pathlib import Path
import pty
import re
import signal
import struct
import subprocess
import tempfile
import termios
import time

os.environ.setdefault('ROS_DOMAIN_ID', '186')
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ.setdefault('ROS_LOG_DIR', tempfile.mkdtemp(prefix='withus-launch-test-'))

import rclpy
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from withus_msgs.msg import DriveCommand, DriverStatus, VehicleStatus

from can_test_support import require_vcan
from test_hardware_drive import ACTUATORS, DrivePeer


class TerminalLaunch:
    """Give one launch its own foreground controlling terminal, without a GUI."""

    def __init__(self, command, log):
        self.master, self.slave = pty.openpty()
        self.original = termios.tcgetattr(self.slave)
        self.resize(80, 24)
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

    def text(self, offset=0):
        return self.output[offset:].decode(errors='replace')

    def resize(self, columns, rows):
        fcntl.ioctl(self.slave, termios.TIOCSWINSZ, struct.pack('HHHH', rows, columns, 0, 0))

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
        self.command = None
        self.sub = self.node.create_subscription(
            DriverStatus, '/withus/driver_status', self.on_status, 10)
        self.vehicle_sub = self.node.create_subscription(
            VehicleStatus, '/withus/vehicle_status', self.on_vehicle, 10)
        self.command_sub = self.node.create_subscription(
            DriveCommand, '/withus/drive_command', self.on_command, 10)
        self.processes = []
        self.terminals = []
        self.checks = 0

    def on_status(self, message):
        self.status = message

    def on_vehicle(self, message):
        self.vehicle = message

    def on_command(self, message):
        self.command = message

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

    def shift(self, tty, key, gear, brake=50):
        screen_offset = len(tty.output)
        since = self.key_target(tty, key, 'key ' + key + ' requests zero-speed gear ' + str(gear),
                                gear=gear, speed=0.0, brake=brake, steering=0.0)
        self.wait(lambda: self.vehicle.gear == gear and self.status.drive_mode_confirmed and
                  not self.status.drive_gear_pending,
                  'fresh gear feedback acknowledges key ' + key)
        name = ('P', 'D', 'N', 'R')[gear]
        self.wait(lambda: name + ' confirmed. Press a new' in tty.text(screen_offset),
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
            terminal.read()
            screen_path = terminal.log_path.with_suffix('.tty')
            screen_path.write_bytes(terminal.output)
            print('Terminal screen capture: ' + str(screen_path), flush=True)
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
    tty, keyboard_log = h.start('withus_examples', 'keyboard_teleop.launch.py',
                                 'key_timeout_sec:=1.0', terminal=True)
    h.wait(lambda: not (termios.tcgetattr(tty.slave)[3] & termios.ICANON),
           'keyboard launch reads its own foreground terminal without a GUI')
    h.wait(lambda: 'Limits: forward ' in tty.text(),
           'keyboard fetches configured limits from the separate driver')
    h.wait(lambda: all(label in tty.text() for label in (
        'Selected speed:', 'Actual gear:', 'Remembered gear:',
        'Command: OFF (no commands published)', 'Last key:', 'Notice:')),
        'fixed terminal display shows speed, gears, command state and notices')
    h.check(b'\x1b[?1049h' in tty.output and b'\x1b[?25l' in tty.output,
            'keyboard uses the alternate terminal screen and hides the cursor')
    h.check('w:' in tty.text() and 'a/d:' in tty.text() and '1=P  2=D  3=N  4=R' in tty.text(),
            'fixed terminal display keeps the keyboard mapping visible')
    h.check('s: speed 0, brake 50%' in tty.text(),
            'default keyboard help shows the configured 50 percent stop brake')
    screen_offset = len(tty.output)
    tty.resize(60, 20)
    h.wait(lambda: 'Selected speed:' in tty.text(screen_offset),
           'fixed display redraws after terminal resize to 60 columns and 20 rows')
    h.pump(0.3)
    redraw = tty.output[screen_offset:]
    positions = re.findall(rb'\x1b\[(\d+);(\d+)H', redraw)
    h.check(positions and all(1 <= int(row) < 20 and 1 <= int(column) < 60
                             for row, column in positions),
            'resized display confines cursor positions to the visible terminal')
    resized_screen = redraw[redraw.rfind(b'\x1b[2J'):]
    rows = re.findall(rb'\x1b\[\d+;1H\x1b\[2K([^\x1b]*)', resized_screen)
    h.check(rows and all(len(row.decode()) < 60 for row in rows),
            'resized display clips every text row before the terminal wrap column')
    h.check(b'\n' not in redraw and b'\r' not in redraw,
            'repeated fixed-screen redraws use no line feeds or growing log lines')
    h.check(tty.process.poll() is None and monitor.poll() is None,
            'keyboard and status clients run simultaneously')
    h.check(sum(name == 'withus_driver' and namespace == '/withus'
                for name, namespace in h.node.get_node_names_and_namespaces()) == 1,
            'client launches do not start another driver')
    h.check({f.can_id for f in h.peer.history()} == {0x501},
            'starting both clients adds no actuator commands before a key')

    h.key_target(tty, 's', 's bootstraps with current gear, 50 percent brake and centered steering',
                 gear=2, speed=0.0, brake=50, steering=0.0)
    h.wait(lambda: h.status.drive_active and h.status.drive_mode_confirmed,
           'fresh full-AD feedback confirms the initial stationary command')
    h.wait(lambda: 'AD confirmed. Press w to drive' in tty.text(),
           'keyboard observes full AD before accepting a gear key')
    h.key_target(tty, 'a', 'a changes only steering while stopped in Neutral',
                 gear=2, speed=0.0, brake=50, steering=0.3)
    h.key_target(tty, 'd', 'd preserves Neutral, zero speed and the explicit brake target',
                 gear=2, speed=0.0, brake=50, steering=-0.3)
    screen_offset = len(tty.output)
    h.shift(tty, '2', 1)
    h.wait(lambda: 'Actual gear: D' in tty.text(screen_offset) and
           'Remembered gear: D' in tty.text(screen_offset) and
           'Command: ACTIVE' in tty.text(screen_offset),
           'fixed display distinguishes actual and remembered D while commands are active')
    since = h.key_target(tty, 'a', 'a after one D key steers without starting forward motion',
                         gear=1, speed=0.0, brake=50, steering=0.3)
    h.key_target(tty, 'd', 'd after one D key preserves zero speed and 50 percent brake',
                 gear=1, speed=0.0, brake=50, steering=-0.3)
    h.check(all(int.from_bytes(f.data[6:8], 'little') == 0
                for f in h.frames(since, 0x504)),
            'steering-only commands never transmit a positive speed target')
    h.key_target(tty, 'w', 'fresh w requests 0.1 m/s while preserving D and right steering',
                 gear=1, speed=0.1, brake=0, steering=-0.3)
    screen_offset = len(tty.output)
    h.key_target(tty, 'q', 'q raises speed by 0.1 m/s without changing D or steering',
                 gear=1, speed=0.2, brake=0, steering=-0.3)
    h.wait(lambda: 'Selected speed: 0.20 m/s' in tty.text(screen_offset),
           'q updates the selected speed in the fixed display')
    h.key_target(tty, 'z', 'z lowers speed by 0.1 m/s without changing D or steering',
                 gear=1, speed=0.1, brake=0, steering=-0.3)
    h.key_target(tty, 'a', 'a while driving changes only steering',
                 gear=1, speed=0.1, brake=0, steering=0.3)
    h.key_target(tty, 'd', 'd while driving preserves speed, brake and D gear',
                 gear=1, speed=0.1, brake=0, steering=-0.3)
    h.key_target(tty, 's', 's preserves D while requesting zero speed, brake and centered steering',
                 gear=1, speed=0.0, brake=50, steering=0.0)

    # Arrow escape sequences must not become their alphabetic suffix commands.
    since = time.monotonic()
    tty.send(b'\x1b[A\x1b[B\x1b[C\x1b[D')
    h.pump(0.15)
    h.check(h.target(since, speed=0.0, brake=50, steering=0.0),
            'arrow sequences do not trigger movement or steering keys')

    h.key_target(tty, 'w', 'w releases the client brake for a fresh motion request', speed=0.1, brake=0)
    since = time.monotonic()
    timed_out_session = h.status.session_id
    screen_offset = len(tty.output)
    h.wait(lambda: h.status.session_id != timed_out_session and not h.status.drive_active,
           'one second without input ends publishing and expires the drive session', 3.0)
    h.wait(lambda: h.status.drive_ready and not h.status.drive_active,
           'command watchdog completes bounded invalidation after keyboard input timeout')
    h.check('Command: OFF (no commands published)' in tty.text(screen_offset),
            'idle timeout visibly stops command publication in the fixed display')
    h.check(all(f.data[1] == 0 for f in h.frames(since, 0x503)),
            'input timeout never inserts an automatic 50 percent brake command')
    h.check(all(any(f.can_id == can_id for f in h.frames(since, valid=False))
                for can_id in ACTUATORS),
            'driver watchdog invalidates all three actuators after client publication ends')
    h.pump(0.15)
    h.check(not h.frames(time.monotonic() - 0.1),
            'keyboard input timeout never automatically rearms or replays motion')
    # Model a vehicle that returns to Neutral after all command publication ends.
    # Remembered gear is only client intent; it must not silently resume TX.
    h.peer.inject(gear=2)
    h.wait(lambda: h.vehicle.gear == 2, 'idle vehicle feedback changes to Neutral')
    h.pump(0.15)
    h.check(not h.frames(time.monotonic() - 0.1),
            'remembered D gear is never restored while keyboard publication is idle')
    screen_offset = len(tty.output)
    since = h.key_target(tty, 's', 'explicit s first rearms using observed Neutral at zero speed',
                         gear=2, speed=0.0, brake=50, steering=0.0)
    h.wait(lambda: h.status.drive_mode_confirmed, 'rearmed session confirms AD again')
    h.wait(lambda: h.vehicle.gear == 1 and not h.status.drive_gear_pending and
           'D confirmed. Press a new' in tty.text(screen_offset),
           'explicit rearm restores the previously confirmed D without another gear key')
    h.check(all(int.from_bytes(f.data[6:8], 'little') == 0
                for f in h.frames(since, 0x504)),
            'remembered-gear restoration never replays the pre-timeout positive speed')
    h.key_target(tty, 'a', 'steering after remembered D restoration still leaves speed zero',
                 gear=1, speed=0.0, brake=50, steering=0.3)
    h.key_target(tty, 'd', 'opposite steering after rearm preserves D and the brake target',
                 gear=1, speed=0.0, brake=50, steering=-0.3)
    h.key_target(tty, 'w', 'only a fresh w resumes motion after idle gear restoration',
                 gear=1, speed=0.1, brake=0, steering=-0.3)
    h.key_target(tty, 's', 'explicit s again centers steering and stops the restored D target',
                 gear=1, speed=0.0, brake=50, steering=0.0)

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
        name = ('P', 'D', 'N', 'R')[gear]
        h.key_target(tty, 'a', 'one ' + name + ' selection survives a steering-only key',
                     gear=gear, speed=0.0, brake=50, steering=0.3)
        h.key_target(tty, 'd', 'one ' + name + ' selection survives the opposite steering key',
                     gear=gear, speed=0.0, brake=50, steering=-0.3)
        h.key_target(tty, 'q', 'q preserves stopped speed, brake, steering and ' + name,
                     gear=gear, speed=0.0, brake=50, steering=-0.3)
        h.key_target(tty, 'z', 'z preserves stopped speed, brake, steering and ' + name,
                     gear=gear, speed=0.0, brake=50, steering=-0.3)
        if gear in (0, 2):
            h.key_target(tty, 'w', 'w cannot change ' + name + ' or start motion in that gear',
                         gear=gear, speed=0.0, brake=50, steering=-0.3)
        h.key_target(tty, 's', 's preserves the gear selected by one ' + key + ' key',
                     gear=gear, speed=0.0, brake=50, steering=0.0)
    h.key_target(tty, 'a', 'Reverse steering alone still requests zero speed and 50 percent brake',
                 gear=3, speed=0.0, brake=50, steering=0.3)
    h.key_target(tty, 'w', 'w uses reverse gear and preserves the selected steering',
                 gear=3, speed=0.1, brake=0, steering=0.3)

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
                 gear=3, speed=0.0, brake=50, steering=0.0)
    h.wait(lambda: h.status.drive_mode_confirmed, 'new session again confirms full AD feedback')

    h.pump(0.06)
    h.key_target(tty, 'w', 'fresh w resumes only after explicit rearm and mode confirmation',
                 gear=3, speed=0.5, brake=0, steering=0.0)
    h.check(b'\n' not in tty.output and b'\r' not in tty.output,
            'control interaction redraws never scroll the terminal with log lines')
    since = time.monotonic()
    tty.send(b'\x03')
    h.wait(lambda: tty.process.poll() is not None, 'Ctrl+C ends keyboard command publication', 5.0)
    h.check(tty.process.returncode == 0, 'keyboard launch exits cleanly after Ctrl+C')
    h.check(termios.tcgetattr(tty.slave) == tty.original, 'Ctrl+C restores original terminal settings')
    tty.read()
    h.check(b'\x1b[?25h' in tty.output and b'\x1b[?1049l' in tty.output,
            'Ctrl+C restores the cursor and previous terminal screen')
    h.check(all(f.data[1] == 0 for f in h.frames(since, 0x503)),
            'Ctrl+C never inserts an automatic brake command')
    h.wait(lambda: h.status.drive_ready and not h.status.drive_active,
           'driver returns to ready after client command expiry and invalidation')
    h.check(driver.poll() is None and monitor.poll() is None and h.status.control_requested,
            'keyboard exit leaves the driver, status monitor and 501 running')

    # A launch override must reach the ROS message and actual brake CAN payload.
    custom, _ = h.start('withus_examples', 'keyboard_teleop.launch.py',
                        'key_timeout_sec:=1.0', 'stop_brake_percent:=30.0', terminal=True)
    h.wait(lambda: 'Limits: forward ' in custom.text() and
           's: speed 0, brake 30%' in custom.text(),
           'custom launch exposes 30 percent brake in the fixed keyboard help')
    # This fresh client must independently observe the configured 200 ms standstill.
    h.pump(0.3)
    command_stamp = h.node.get_clock().now().nanoseconds
    h.key_target(custom, 's', 'custom brake applies to initial zero-speed preparation on CAN',
                 gear=3, speed=0.0, brake=30, steering=0.0)
    h.wait(lambda: h.command is not None and
           h.command.header.stamp.sec * 1000000000 + h.command.header.stamp.nanosec > command_stamp and
           h.command.gear == 3 and h.command.speed_mps == 0.0 and h.command.brake_percent == 30.0,
           'custom stop brake is published as 30 percent in the ROS DriveCommand')
    h.wait(lambda: h.status.drive_mode_confirmed and
           'AD confirmed. Press w to drive' in custom.text(),
           'custom-brake client confirms AD using its stationary command')
    h.wait(lambda: 'Target steering: 0.000 rad | brake: 30%' in custom.text(),
           'fixed display reports the actual 30 percent command target')
    h.shift(custom, '2', 1, brake=30)
    h.key_target(custom, 'a', 'custom brake survives a steering-only input after shifting',
                 gear=1, speed=0.0, brake=30, steering=0.3)
    h.key_target(custom, 'w', 'positive speed releases the custom stop brake to zero',
                 gear=1, speed=0.1, brake=0, steering=0.3)
    command_stamp = h.node.get_clock().now().nanoseconds
    h.key_target(custom, 's', 'explicit s uses custom 30 percent brake and centers steering',
                 gear=1, speed=0.0, brake=30, steering=0.0)
    h.wait(lambda: h.command is not None and
           h.command.header.stamp.sec * 1000000000 + h.command.header.stamp.nanosec > command_stamp and
           h.command.gear == 1 and h.command.speed_mps == 0.0 and h.command.brake_percent == 30.0,
           'explicit custom stop reaches the ROS DriveCommand with zero speed')
    custom.send(b'\x03')
    h.wait(lambda: custom.process.poll() is not None, 'custom-brake keyboard exits after Ctrl+C', 5.0)
    h.check(custom.process.returncode == 0 and
            termios.tcgetattr(custom.slave) == custom.original,
            'custom-brake keyboard exits cleanly and restores its terminal')
    h.wait(lambda: h.status.drive_ready and not h.status.drive_active,
           'driver stays running and returns to ready after the custom client exits')
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

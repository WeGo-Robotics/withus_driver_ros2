"""Keyboard client for the hardware_drive DriveCommand interface."""

import argparse
from contextlib import ExitStack
from dataclasses import dataclass
import errno
import math
import os
import select
import signal
import sys
import termios
import time
import tty


PARAMETER_NAMES = (
    'max_forward_speed_mps', 'max_reverse_speed_mps',
    'max_steering_angle_rad', 'stop_confirm_duration_ms',
    'stop_speed_threshold_mps',
)
BLOCKING_FIELDS = (
    'emergency_stop_pressed', 'motor_fault', 'remote_fault', 'dbs_fault',
    'bms_fault', 'eps_fault', 'front_bumper_triggered',
    'rear_bumper_triggered', 'parallel_driving_active',
)
GEAR_KEYS = {'1': 0, '2': 1, '3': 2, '4': 3}
GEAR_NAMES = ('P', 'D', 'N', 'R')
DEFAULT_STOP_BRAKE_PERCENT = 50.0
HELP = """
Withus keyboard client (hardware_drive)
  w: selected speed (keep steering)
  a / d: maximum left / right steering (keep speed, brake, and gear)
  s: speed 0, brake {stop_brake_percent:g}%, steering center (also prepares control)
  q / z: increase / decrease selected speed
  1 / 2 / 3 / 4: P / D / N / R (stop before changing gear)
  Ctrl+C: stop publishing and exit (does not send a brake command)
Start with s or a gear key; wait for AD/gear confirmation, then press w to drive or a/d to steer.
Only run one command client. The status monitor may run in another terminal.
"""


def stamp_ns(stamp):
    """Convert a ROS builtin time without requiring ROS in the policy tests."""
    return stamp.sec * 1_000_000_000 + stamp.nanosec


@dataclass(frozen=True)
class Limits:
    """Read-only limits fetched from the running driver."""

    forward: float
    reverse: float
    steering: float
    stopped_dwell: float
    stopped_speed: float

    def __post_init__(self):
        if not all(math.isfinite(value) and value > 0.0 for value in (
                self.forward, self.reverse, self.steering,
                self.stopped_dwell, self.stopped_speed)):
            raise ValueError('Driver limits must be finite and positive.')

    @classmethod
    def from_parameters(cls, values):
        """Decode exact numeric parameter types, rejecting unavailable values."""
        if len(values) != len(PARAMETER_NAMES):
            raise ValueError('Driver returned an incomplete parameter response.')
        numbers = []
        for name, value in zip(PARAMETER_NAMES, values):
            if value.type == 2:  # rcl_interfaces/ParameterType.PARAMETER_INTEGER
                number = float(value.integer_value)
            elif value.type == 3:  # PARAMETER_DOUBLE
                number = value.double_value
            else:
                raise ValueError(f'Driver parameter {name} is not numeric.')
            numbers.append(number)
        numbers[3] /= 1000.0
        return cls(*numbers)


@dataclass(frozen=True)
class Target:
    """One complete client target; reverse uses gear and positive speed."""

    gear: int
    speed: float = 0.0
    steering: float = 0.0
    brake: float = DEFAULT_STOP_BRAKE_PERCENT


class TeleopController:
    """Keep keyboard intent separate from ROS, terminal handling, and feedback."""

    def __init__(self, initial_speed=0.1, speed_step=0.1,
                 key_timeout=1.0, left_sign=1,
                 stop_brake_percent=DEFAULT_STOP_BRAKE_PERCENT):
        if (not math.isfinite(initial_speed) or initial_speed < 0.0 or
                not math.isfinite(speed_step) or speed_step <= 0.0 or
                not math.isfinite(key_timeout) or key_timeout <= 0.0 or
                left_sign not in (-1, 1)):
            raise ValueError('Invalid keyboard speed, timeout, or steering sign.')
        if not math.isfinite(stop_brake_percent) or not 0.0 <= stop_brake_percent <= 100.0:
            raise ValueError('stop_brake_percent must be finite and between 0 and 100.')
        self.selected_speed = initial_speed
        self.speed_step = speed_step
        self.key_timeout = key_timeout
        self.left_sign = left_sign
        self.stop_brake_percent = stop_brake_percent
        self.limits = None
        self.driver = self.vehicle = None
        self.driver_at = self.vehicle_at = 0.0
        self.session = None
        self.target = None
        self.selected_gear = None  # Last confirmed choice; never an active command.
        self.phase = 'idle'
        self.notice = 'Waiting for driver limits and fresh status.'
        self.stopped_since = None
        self.started_at = 0.0
        self.last_command_key = 0.0
        self.last_ros_ns = 0
        self.saw_active = False
        self.confirmed = False
        self.queued_gear = None
        self.gear_request_ns = None

    def set_limits(self, limits):
        """Install fetched limits before accepting any key that can command."""
        self.limits = limits
        self.selected_speed = min(self.selected_speed, limits.forward)
        self.notice = ('Limits received. Select A and press s or a gear key '
                       'when the driver is ready.')

    def on_driver(self, message, now):
        """Record driver state and a local monotonic arrival time."""
        self.driver, self.driver_at = message, now

    def on_vehicle(self, message, now):
        """Record vehicle feedback and a local monotonic arrival time."""
        self.vehicle, self.vehicle_at = message, now

    def feedback_problem(self, now, ros_ns):
        """Reject delayed ROS messages as well as stale individual CAN IDs."""
        if self.limits is None:
            return 'Waiting for driver limits.'
        if self.driver is None or self.vehicle is None:
            return 'Waiting for driver_status and vehicle_status.'
        if ros_ns <= 0:
            return 'ROS time is not valid.'
        for message, arrival in ((self.driver, self.driver_at),
                                 (self.vehicle, self.vehicle_at)):
            age_ns = ros_ns - stamp_ns(message.header.stamp)
            if (now - arrival > 0.25 or age_ns > 250_000_000 or
                    age_ns < -50_000_000 or stamp_ns(message.header.stamp) <= 0):
                return 'ROS status is missing, delayed, or stale.'
        driver, vehicle = self.driver, self.vehicle
        if driver.control_profile != 'hardware_drive':
            return 'Keyboard client requires the hardware_drive profile.'
        if not (driver.can_connected and driver.enable_tx and
                driver.control_requested and driver.session_id != 0):
            return 'Driver CAN transmission or mode request is unavailable.'
        if not all(getattr(vehicle, f'status_{can_id}_valid')
                   for can_id in ('301', '303', '304', '10a')):
            return 'Vehicle or A-switch CAN feedback is stale.'
        if vehicle.remote_a_raw != 1 or not vehicle.remote_a_selected:
            return 'A switch is not selected.'
        if any(getattr(vehicle, field) for field in BLOCKING_FIELDS):
            return 'Vehicle feedback reports a blocking condition.'
        if vehicle.gear not in (0, 1, 2, 3):
            return 'Vehicle gear is unknown.'
        if not (math.isfinite(vehicle.speed_mps) and
                math.isfinite(vehicle.steering_angle_rad)):
            return 'Vehicle speed or steering is not finite.'
        if vehicle.drive_mode not in (1, 3):
            return 'Vehicle mode must be AD (1) or semi-autonomous (3).'
        return ''

    def discard(self, reason, remember_gear=False):
        """Clear commands; normal input expiry may retain a confirmed selection."""
        if not remember_gear:
            self.selected_gear = None
        self.session = self.target = self.queued_gear = None
        self.gear_request_ns = None
        self.phase = 'idle'
        self.saw_active = self.confirmed = False
        if self.selected_gear is not None:
            reason += f' Remembered gear: {GEAR_NAMES[self.selected_gear]}.'
        self.notice = reason + ' Press s or a gear key again after recovery.'

    def stopped(self, now):
        """Require measured speed to remain inside the driver's stop threshold."""
        return (self.stopped_since is not None and
                now - self.stopped_since >= self.limits.stopped_dwell)

    def speed_limit(self, gear=None):
        """Use the selected gear's configured speed limit."""
        if gear is None:
            gear = self.target.gear if self.target else self.vehicle.gear
        return self.limits.reverse if gear == 3 else self.limits.forward

    def stopped_target(self, gear):
        """Use the configured brake for stop, preparation, and gear commands."""
        return Target(gear, brake=self.stop_brake_percent)

    def request_stop(self, reason=None):
        """Preserve an in-flight gear request while stopping and centering."""
        if self.target is None:
            return False
        self.target = self.stopped_target(self.target.gear)
        if self.phase in ('driving', 'steering', 'waiting_gear'):
            self.phase = 'holding'
        if self.phase in ('preparing', 'holding'):
            if self.queued_gear is not None:
                self.selected_gear = self.target.gear
            self.queued_gear = None
        self.notice = reason if reason is not None else (
            f'Stop requested: brake {self.stop_brake_percent:g}%, steering centered.')
        return True

    def _start_gear(self, gear, ros_ns):
        self.target = self.stopped_target(gear)
        self.phase = 'gear'
        self.gear_request_ns = ros_ns
        self.queued_gear = None
        self.notice = f'Waiting for new {GEAR_NAMES[gear]} gear feedback; speed remains 0.'

    def refresh(self, now, ros_ns):
        """Advance feedback handshakes; never begin motion from remembered keys."""
        backwards = self.last_ros_ns and ros_ns < self.last_ros_ns
        self.last_ros_ns = ros_ns
        problem = self.feedback_problem(now, ros_ns)
        if backwards:
            problem = 'ROS time moved backwards.'
        if problem:
            self.stopped_since = None
            if self.session is not None:
                self.discard(problem)
            else:
                self.selected_gear = None
                self.notice = problem
            return False
        if abs(self.vehicle.speed_mps) <= self.limits.stopped_speed:
            if self.stopped_since is None:
                self.stopped_since = now
        else:
            self.stopped_since = None
        if self.session is None:
            return True
        driver, vehicle = self.driver, self.vehicle
        if driver.session_id != self.session:
            self.discard('Driver session changed; old commands were discarded.')
            return False
        if not driver.drive_ready:
            self.discard(driver.drive_reason or 'Driver readiness was lost.')
            return False
        if ((self.saw_active and not driver.drive_active) or
                (not self.saw_active and now - self.started_at >= 0.4)):
            self.discard(driver.last_rejection or 'Driver did not keep the command active.')
            return False
        self.saw_active = self.saw_active or driver.drive_active
        if self.confirmed and not (driver.drive_mode_confirmed and
                                   vehicle.drive_mode == 1 and
                                   vehicle.ad_signal_received):
            self.discard('Confirmed AD mode was lost.')
            return False
        if self.phase != 'gear' and vehicle.gear != self.target.gear:
            self.discard('Gear changed without this client requesting it.')
            return False
        if (self.phase == 'preparing' and driver.drive_active and
                driver.drive_mode_confirmed and vehicle.drive_mode == 1 and
                vehicle.ad_signal_received):
            self.confirmed = True
            self.phase = 'holding'
            self.notice = 'AD confirmed. Press w to drive, a/d to steer, or select a gear.'
            if self.queued_gear is not None:
                self.phase = 'waiting_gear'
            else:
                self.selected_gear = self.target.gear
        if self.phase == 'waiting_gear':
            if self.stopped(now):
                self._start_gear(self.queued_gear, ros_ns)
            else:
                self.notice = ('AD confirmed. Keeping the selected gear request while '
                               'waiting for measured standstill.')
        if self.phase == 'gear' and (
                vehicle.gear == self.target.gear and
                stamp_ns(vehicle.status_303_stamp) > self.gear_request_ns and
                stamp_ns(driver.header.stamp) > self.gear_request_ns and
                stamp_ns(driver.header.stamp) >= stamp_ns(vehicle.status_303_stamp) and
                driver.drive_mode_confirmed and not driver.drive_gear_pending):
            self.phase = 'holding'
            self.gear_request_ns = None
            self.selected_gear = self.target.gear
            self.selected_speed = min(self.selected_speed, self.speed_limit())
            self.notice = (f'{GEAR_NAMES[self.target.gear]} confirmed. '
                           'Press a new w key to drive; a/d only changes steering.')
        if self.phase != 'gear' and driver.drive_gear_pending:
            self.discard('An unexpected gear request is pending in the driver.')
            return False
        if now - self.last_command_key >= self.key_timeout:
            self.discard('Keyboard input timed out: command publishing stopped; '
                         'no brake was requested.', remember_gear=True)
            return False
        return True

    def _prepare(self, now, requested_gear=None):
        if not self.driver.drive_ready or self.driver.drive_active:
            self.notice = (self.driver.drive_reason if not self.driver.drive_ready else
                           'Another drive command is already active.')
            return
        if abs(self.vehicle.speed_mps) > self.limits.stopped_speed:
            self.notice = 'A new session requires measured standstill.'
            return
        if self.selected_gear is None:
            self.selected_gear = self.vehicle.gear
        desired_gear = self.selected_gear if requested_gear is None else requested_gear
        self.session = self.driver.session_id
        self.target = self.stopped_target(self.vehicle.gear)
        self.phase = 'preparing'
        self.started_at = now
        self.last_command_key = now
        self.saw_active = self.confirmed = False
        self.queued_gear = desired_gear if desired_gear != self.vehicle.gear else None
        self.notice = (f'Sending zero speed and brake {self.stop_brake_percent:g}%; '
                       'waiting for full AD confirmation.')
        if self.queued_gear is not None:
            self.notice += f' Selected gear after preparation: {GEAR_NAMES[self.queued_gear]}.'

    def handle_key(self, key, now, ros_ns):
        """Apply only a current explicit key; rejected motion is never queued."""
        if key not in 'wasdqz1234':
            return
        if not self.refresh(now, ros_ns):
            return
        if key in 'qz':
            change = self.speed_step if key == 'q' else -self.speed_step
            self.selected_speed = min(self.speed_limit(), max(
                0.0, round(self.selected_speed + change, 10)))
            if self.session is not None:
                self.last_command_key = now
            if self.phase == 'driving':
                self.target = Target(self.target.gear, self.selected_speed,
                                     self.target.steering, 0.0)
                if self.selected_speed == 0.0:
                    self.phase = 'steering' if self.target.steering != 0.0 else 'holding'
            self.notice = f'Selected speed: {self.selected_speed:.3f} m/s.'
            return
        if key == 's':
            if self.session is None:
                self._prepare(now)
            else:
                self.request_stop()
                self.last_command_key = now
            return
        if key in GEAR_KEYS:
            gear = GEAR_KEYS[key]
            if self.session is None:
                self._prepare(now, gear)
            elif self.phase == 'gear':
                if gear == self.target.gear:
                    self.last_command_key = now
                self.notice = 'Wait for the pending gear; s remains available.'
            elif self.phase in ('preparing', 'waiting_gear'):
                self.queued_gear = gear if gear != self.target.gear else None
                self.last_command_key = now
                if self.queued_gear is None:
                    self.selected_gear = self.target.gear
                if self.confirmed and self.queued_gear is None:
                    self.phase = 'holding'
                self.notice = 'Gear selected; waiting for AD and standstill at speed 0.'
            elif not self.stopped(now):
                self.notice = 'Gear change refused: press s and wait for measured standstill.'
            elif gear == self.target.gear:
                self.request_stop()
                self.last_command_key = now
            else:
                self._start_gear(gear, ros_ns)
                self.last_command_key = now
            return
        if self.phase not in ('holding', 'driving', 'steering') or not self.confirmed:
            self.notice = ('Motion key ignored. Press s or a gear key, '
                           'then wait for AD/gear confirmation.')
            return
        if key in 'ad':
            steering = self.limits.steering * self.left_sign * (1 if key == 'a' else -1)
            self.target = Target(self.target.gear, self.target.speed,
                                 steering, self.target.brake)
        else:  # w changes longitudinal intent without resetting steering or gear.
            if self.target.gear not in (1, 3) and self.selected_speed > 0.0:
                self.notice = 'Motion key ignored. Select D or R first.'
                return
            self.selected_speed = min(self.selected_speed, self.speed_limit())
            brake = self.target.brake if self.selected_speed == 0.0 else 0.0
            self.target = Target(self.target.gear, self.selected_speed,
                                 self.target.steering, brake)
        self.phase = 'driving' if self.target.speed > 0.0 else 'steering'
        self.last_command_key = now
        self.notice = (f'{GEAR_NAMES[self.target.gear]}: speed={self.target.speed:.3f} m/s, '
                       f'steering={self.target.steering:.3f} rad, '
                       f'brake={self.target.brake:g}% (selected speed={self.selected_speed:.3f}).')

    def tick(self, now, ros_ns):
        """Return the current whole target only while this session remains valid."""
        if not self.refresh(now, ros_ns):
            return None
        return self.target


class KeyParser:
    """Ignore escape sequences, arrow keys, and bracketed pasted text."""

    def __init__(self):
        self.state = 'normal'
        self.sequence = ''
        self.paste_tail = ''

    def feed(self, data):
        """Return only deliberate single-character command keys and Ctrl+C."""
        result = []
        for byte in data:
            char = chr(byte)
            if char == '\x03':
                result.append(char)
                continue
            if self.state == 'paste':
                self.paste_tail = (self.paste_tail + char)[-6:]
                if self.paste_tail == '\x1b[201~':
                    self.state = 'normal'
                continue
            if self.state == 'escape':
                self.state = ('csi' if char == '[' else
                              'ss3' if char == 'O' else 'normal')
                self.sequence = ''
                continue
            if self.state == 'ss3':
                self.state = 'normal'
                continue
            if self.state == 'csi':
                self.sequence += char
                if '@' <= char <= '~':
                    self.state = 'paste' if self.sequence == '200~' else 'normal'
                    self.paste_tail = ''
                elif len(self.sequence) > 64:
                    self.state = 'normal'
                continue
            if char == '\x1b':
                self.state = 'escape'
            elif char in 'wasdqz1234':
                result.append(char)
        return result


class TerminalInput:
    """Own a real terminal even when ROS launch gives the child a pipe stdin."""

    def __init__(self, terminal_fd=None):
        self.fd = None
        self.saved = None
        self.parser = KeyParser()
        self.eof = False
        if terminal_fd is not None:
            self.fd = os.dup(terminal_fd)
        else:
            try:
                self.fd = os.open('/dev/tty', os.O_RDWR | os.O_NONBLOCK)
            except OSError:
                if sys.stdin.isatty():
                    self.fd = os.dup(sys.stdin.fileno())
        if self.fd is None or not os.isatty(self.fd):
            if self.fd is not None:
                os.close(self.fd)
            raise RuntimeError('Keyboard teleop requires an interactive terminal.')
        if os.tcgetpgrp(self.fd) != os.getpgrp():
            os.close(self.fd)
            self.fd = None
            raise RuntimeError('Keyboard teleop must run in the foreground terminal.')
        try:
            self.saved = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)
            os.write(self.fd, b'\x1b[?2004h')
        except BaseException:
            self.close()
            raise

    def read(self):
        """Read without blocking the 20 ms ROS publication timer."""
        if not select.select([self.fd], [], [], 0.0)[0]:
            return []
        try:
            data = os.read(self.fd, 1024)
        except BlockingIOError:
            return []
        if not data:
            self.eof = True
        return self.parser.feed(data)

    def close(self):
        """Restore terminal settings after normal exit, signals, or exceptions."""
        if self.fd is None:
            return
        try:
            if self.saved is not None:
                termios.tcsetattr(self.fd, termios.TCSANOW, self.saved)
                os.write(self.fd, b'\x1b[?2004l')
        except (OSError, termios.error) as error:
            # A closed terminal cannot be restored, but must not prevent ROS cleanup.
            if error.args[0] not in (errno.EIO, errno.ENXIO):
                raise
        finally:
            os.close(self.fd)
            self.fd = None


def create_node(options):
    """Build the ROS adapter lazily so the controller can be tested without ROS."""
    from rcl_interfaces.srv import GetParameters
    from rclpy.clock import Clock, ClockType
    from rclpy.node import Node
    from withus_msgs.msg import DriveCommand, DriverStatus, VehicleStatus

    class KeyboardTeleop(Node):
        """Publish current keyboard targets and subscribe to their feedback."""

        def __init__(self):
            super().__init__('keyboard_teleop', namespace=options.namespace)
            self.controller = TeleopController(
                options.initial_speed, options.speed_step,
                options.key_timeout_sec, options.left_steering_sign,
                options.stop_brake_percent)
            self.publisher = self.create_publisher(DriveCommand, 'drive_command', 1)
            self.driver_subscription = self.create_subscription(
                DriverStatus, 'driver_status', self.on_driver, 1)
            self.vehicle_subscription = self.create_subscription(
                VehicleStatus, 'vehicle_status', self.on_vehicle, 1)
            driver_name = (options.driver_node or
                           self.get_namespace().rstrip('/') + '/withus_driver')
            self.parameter_client = self.create_client(
                GetParameters, driver_name.rstrip('/') + '/get_parameters')
            self.parameter_future = None
            self.parameter_requested_at = 0.0
            self.last_notice = ''
            self.last_stamp = 0
            self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
            self.timer = self.create_timer(0.02, self.on_timer, clock=self.steady_clock)
            self.get_logger().info(HELP.format(
                stop_brake_percent=self.controller.stop_brake_percent))
            self.get_logger().info(
                f'Input timeout={options.key_timeout_sec:g} s; '
                f'left steering sign={options.left_steering_sign:+d}. '
                'No drive commands are sent until s or a gear key.')

        def on_driver(self, message):
            self.controller.on_driver(message, time.monotonic())

        def on_vehicle(self, message):
            self.controller.on_vehicle(message, time.monotonic())

        def read_limits(self, now):
            if self.controller.limits is not None:
                return
            if self.parameter_future is not None and self.parameter_future.done():
                try:
                    response = self.parameter_future.result()
                    limits = Limits.from_parameters(response.values)
                    self.controller.set_limits(limits)
                    self.get_logger().info(
                        f'Driver limits: forward={limits.forward:g} m/s, '
                        f'reverse={limits.reverse:g} m/s, '
                        f'steering=+/-{limits.steering:g} rad.')
                except (ValueError, AttributeError, RuntimeError) as error:
                    self.controller.notice = f'Cannot read driver limits: {error}'
                self.parameter_future = None
            if self.controller.limits is not None:
                return
            if self.parameter_future is not None and now - self.parameter_requested_at > 3.0:
                self.parameter_future.cancel()
                self.parameter_future = None
            if (self.parameter_future is None and
                    now - self.parameter_requested_at >= 1.0 and
                    self.parameter_client.service_is_ready()):
                request = GetParameters.Request()
                request.names = list(PARAMETER_NAMES)
                self.parameter_future = self.parameter_client.call_async(request)
                self.parameter_requested_at = now

        def report(self):
            if self.controller.notice != self.last_notice:
                self.get_logger().info(self.controller.notice)
                self.last_notice = self.controller.notice

        def key(self, key):
            self.controller.handle_key(
                key, time.monotonic(), self.get_clock().now().nanoseconds)
            self.report()

        def stop_for_exit(self):
            self.controller.discard('Client exiting: command publishing stopped.')
            self.timer.cancel()

        def on_timer(self):
            now = time.monotonic()
            self.read_limits(now)
            stamp = self.get_clock().now()
            target = self.controller.tick(now, stamp.nanoseconds)
            self.report()
            if target is None or stamp.nanoseconds <= self.last_stamp:
                return
            command = DriveCommand()
            command.header.stamp = stamp.to_msg()
            command.header.frame_id = self.controller.vehicle.header.frame_id
            command.session_id = self.controller.session
            command.gear = target.gear
            command.speed_mps = target.speed
            command.steering_angle_rad = target.steering
            command.brake_percent = target.brake
            self.publisher.publish(command)
            self.last_stamp = stamp.nanoseconds

    return KeyboardTeleop()


def main(args=None):
    """Run a keyboard client and stop publishing on terminal exit."""
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.signals import SignalHandlerOptions
    from rclpy.utilities import remove_ros_args

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--namespace', default='withus')
    parser.add_argument('--driver-node', default='')
    parser.add_argument('--initial-speed', type=float, default=0.1)
    parser.add_argument('--speed-step', type=float, default=0.1)
    parser.add_argument('--stop-brake-percent', type=float, default=DEFAULT_STOP_BRAKE_PERCENT,
                        help='Brake request for s, preparation, and gear changes (0..100%%).')
    parser.add_argument('--key-timeout-sec', type=float, default=1.0)
    parser.add_argument('--left-steering-sign', type=int, choices=(-1, 1), default=1)
    parser.add_argument('--terminal-fd', type=int)
    command_line = sys.argv if args is None else [sys.argv[0], *args]
    options = parser.parse_args(remove_ros_args(args=command_line)[1:])
    try:
        TeleopController(options.initial_speed, options.speed_step,
                         options.key_timeout_sec, options.left_steering_sign,
                         options.stop_brake_percent)
    except ValueError as error:
        parser.error(str(error))
    from withus_examples.teleop_screen import TerminalScreen

    previous_handlers = {signum: signal.getsignal(signum)
                         for signum in (signal.SIGINT, signal.SIGTERM)}
    stop_requested = False

    def interrupted(_signum, _frame):
        nonlocal stop_requested
        stop_requested = True

    def restore_handlers():
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)

    def shutdown_ros():
        if rclpy.ok():
            rclpy.shutdown()

    # Keep ROS logs in their files and /rosout, away from the fixed terminal view.
    ros_args = [command_line[0], '--ros-args', '--disable-stdout-logs', '--',
                *command_line[1:]]
    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    try:
        with ExitStack() as cleanup:
            cleanup.callback(restore_handlers)
            cleanup.callback(shutdown_ros)
            for signum in previous_handlers:
                signal.signal(signum, interrupted)
            terminal = TerminalInput(options.terminal_fd)
            cleanup.callback(terminal.close)
            node = create_node(options)
            cleanup.callback(node.destroy_node)
            screen = TerminalScreen(terminal.fd)
            cleanup.callback(screen.close)
            cleanup.callback(node.stop_for_exit)
            last_key = '-'
            next_display = 0.0
            while rclpy.ok() and not stop_requested:
                rclpy.spin_once(node, timeout_sec=0.01)
                if stop_requested:
                    break
                for key in terminal.read():
                    if key == '\x03':
                        stop_requested = True
                        break
                    last_key = key
                    node.key(key)
                if terminal.eof or stop_requested:
                    break
                now = time.monotonic()
                if now >= next_display:
                    screen.render(node.controller, now, node.get_clock().now().nanoseconds,
                                  last_key=last_key)
                    next_display = now + 0.1
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    except (OSError, RuntimeError) as error:
        print(f'Keyboard teleop: {error}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())

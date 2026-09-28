"""Request one bounded stationary brake response and report its feedback."""

import argparse
import math
import sys
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args
from withus_msgs.msg import DriveCommand, DriverStatus, VehicleStatus


def brake_percent(value):
    """Require the caller to choose a finite percentage within the wire range."""
    try:
        result = float(value)
    except ValueError as error:
        raise argparse.ArgumentTypeError('must be a number') from error
    if not math.isfinite(result) or not 0.0 <= result <= 100.0:
        raise argparse.ArgumentTypeError('must be finite and within 0..100')
    return result


def test_duration(value):
    """Bound this single stationary client request to at most 1.5 seconds."""
    try:
        result = float(value)
    except ValueError as error:
        raise argparse.ArgumentTypeError('must be a number') from error
    if not math.isfinite(result) or not 0.0 < result <= 1.5:
        raise argparse.ArgumentTypeError('must be finite and within (0, 1.5] seconds')
    return result


def stationary_problem(vehicle):
    """Require fresh, selected-A, neutral, stopped feedback for this example."""
    if not all(getattr(vehicle, f'status_{can_id}_valid')
               for can_id in ('301', '303', '304', '10a')):
        return 'Vehicle or A-switch CAN feedback is stale or missing.'
    if not vehicle.remote_a_selected or vehicle.remote_a_raw != 1:
        return 'A switch must be selected.'
    if vehicle.gear != VehicleStatus.GEAR_N:
        return 'Stationary brake response requires Neutral gear.'
    if (not math.isfinite(vehicle.speed_mps) or
            abs(vehicle.speed_mps) > 0.05):
        return 'Stationary brake response requires |speed| <= 0.05 m/s.'
    if (not math.isfinite(vehicle.steering_angle_rad) or
            abs(vehicle.steering_angle_rad) > 0.3):
        return ("Observed steering exceeds this example's finite +/-0.3 rad limit. "
                'Align steering before starting; the client will not center it.')
    if vehicle.drive_mode not in (
            VehicleStatus.DRIVE_MODE_AD,
            VehicleStatus.DRIVE_MODE_SEMI_AUTONOMOUS):
        return 'Observed drive mode must be AD (1) or semi-autonomous (3).'
    blocked_fields = (
        'emergency_stop_pressed', 'motor_fault', 'remote_fault', 'dbs_fault',
        'bms_fault', 'eps_fault', 'front_bumper_triggered',
        'rear_bumper_triggered', 'parallel_driving_active',
    )
    if any(getattr(vehicle, field) for field in blocked_fields):
        return 'Vehicle feedback reports a blocking condition.'
    return ''


class BrakeResponseCheck(Node):
    """Publish one session's brake commands and observe the reported pressure."""

    def __init__(self, percent, duration, namespace):
        """Create ROS interfaces without opening a CAN socket or calling services."""
        super().__init__('brake_response_check', namespace=namespace)
        self.percent = percent
        self.duration = duration
        self.driver = None
        self.vehicle = None
        self.driver_at = 0.0
        self.vehicle_at = 0.0
        self.phase = 'waiting'
        self.created_at = time.monotonic()
        self.started_at = 0.0
        self.stopped_at = 0.0
        self.report_at = 0.0
        self.session = None
        self.steering_angle_rad = None
        self.saw_active = False
        self.observed_inactive = False
        self.stop_reason = ''
        self.pressures = []
        self.exit_code = 1
        self.done = False
        self.publisher = self.create_publisher(
            DriveCommand, 'drive_command', 1)
        self.driver_subscription = self.create_subscription(
            DriverStatus, 'driver_status', self.on_driver, 1)
        self.vehicle_subscription = self.create_subscription(
            VehicleStatus, 'vehicle_status', self.on_vehicle, 1)
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(
            0.02, self.on_timer, clock=self.steady_clock)
        self.get_logger().info(
            f'Waiting up to 5 s for hardware_drive readiness; '
            f'request={percent:g}%, publish duration={duration:g} s. '
            'This measures a stationary response, not stopping performance.')

    def on_driver(self, message):
        """Record the latest driver status and its local arrival time."""
        self.driver = message
        self.driver_at = time.monotonic()

    def on_vehicle(self, message):
        """Collect fresh pressure samples during this single test and observation."""
        self.vehicle = message
        self.vehicle_at = time.monotonic()
        if (self.phase in ('publishing', 'observing') and
                message.status_304_valid and
                math.isfinite(message.brake_pressure_pa)):
            self.pressures.append(message.brake_pressure_pa)

    def fresh(self, now):
        """Require both ROS streams to have arrived within 250 milliseconds."""
        return (self.driver is not None and self.vehicle is not None and
                now - self.driver_at <= 0.25 and now - self.vehicle_at <= 0.25)

    def stop_publishing(self, reason, completed=False):
        """Stop this session permanently and allow bounded feedback observation."""
        if self.phase in ('observing', 'done'):
            return
        was_publishing = self.phase == 'publishing'
        self.phase = 'observing'
        self.stopped_at = time.monotonic()
        self.stop_reason = reason
        self.exit_code = 0 if completed and self.saw_active else 1
        self.get_logger().info(f'Publishing stopped: {reason}')
        if not was_publishing:
            self.finish()

    def finish(self):
        """Report observations without inferring physical stopping or release."""
        self.phase = 'done'
        self.done = True
        if self.pressures:
            self.get_logger().info(
                f'Observed brake pressure: first={self.pressures[0]:.0f} Pa, '
                f'min={min(self.pressures):.0f} Pa, '
                f'max={max(self.pressures):.0f} Pa, '
                f'last={self.pressures[-1]:.0f} Pa.')
        else:
            self.get_logger().warning('No fresh brake-pressure samples recorded.')
        if self.started_at:
            self.get_logger().info(
                f'Driver active observed={self.saw_active}; '
                f'active=false after publishing stopped={self.observed_inactive}. '
                'These flags describe driver commands, not physical brake release.')
        if not self.observed_inactive and self.started_at:
            self.exit_code = 1

    def on_timer(self):
        """Use steady time for duration, stale-topic detection, and final waiting."""
        if self.done:
            return
        now = time.monotonic()
        fresh = self.fresh(now)
        if self.phase == 'waiting':
            if fresh and self.driver.control_profile != 'hardware_drive':
                self.stop_publishing('Driver is not using hardware_drive.')
                return
            problem = (stationary_problem(self.vehicle) if fresh
                       else 'ROS status is missing or stale.')
            # An out-of-range angle must never turn into an implicit centering
            # command. Refuse this run and let the operator align it explicitly.
            if fresh and (not math.isfinite(self.vehicle.steering_angle_rad) or
                          abs(self.vehicle.steering_angle_rad) > 0.3):
                self.stop_publishing(problem)
                return
            if fresh and self.driver.drive_active:
                self.stop_publishing('Another drive command is already active.')
                return
            if now - self.created_at >= 5.0:
                reason = problem or (self.driver.drive_reason if self.driver
                                     else 'No driver status.')
                self.stop_publishing(f'Readiness timeout: {reason}')
                return
            if not (fresh and not problem and self.driver.drive_ready and
                    self.driver.control_requested and
                    self.driver.can_connected and self.driver.enable_tx and
                    self.driver.session_id != 0 and
                    self.publisher.get_subscription_count() > 0 and
                    self.get_clock().now().nanoseconds > 0):
                return
            self.session = self.driver.session_id
            self.steering_angle_rad = self.vehicle.steering_angle_rad
            self.started_at = now
            self.phase = 'publishing'
            if math.isfinite(self.vehicle.brake_pressure_pa):
                self.pressures.append(self.vehicle.brake_pressure_pa)
            self.get_logger().info(
                f'Starting session {self.session}; '
                f'observed mode={self.vehicle.drive_mode}, '
                f'speed={self.vehicle.speed_mps:.3f} m/s, '
                f'hold steering={self.steering_angle_rad:.3f} rad, '
                f'pressure={self.vehicle.brake_pressure_pa:.0f} Pa.')

        if self.phase == 'publishing':
            if not fresh:
                self.stop_publishing('ROS status is stale.')
                return
            if self.driver.session_id != self.session:
                self.stop_publishing('Driver session changed; this client will not restart.')
                return
            problem = stationary_problem(self.vehicle)
            if problem:
                self.stop_publishing(problem)
                return
            if not (self.driver.control_requested and
                    self.driver.can_connected and self.driver.enable_tx):
                self.stop_publishing('Driver transmission or mode request ended.')
                return
            if self.saw_active and not self.driver.drive_active:
                self.stop_publishing(self.driver.drive_reason or 'Driver ended the test.')
                return
            if not self.driver.drive_ready:
                self.stop_publishing(self.driver.drive_reason or 'Readiness was lost.')
                return
            self.saw_active = self.saw_active or self.driver.drive_active
            if now - self.started_at >= self.duration:
                self.stop_publishing('Requested publish duration completed.', completed=True)
                return
            if not self.saw_active and now - self.started_at >= 0.3:
                self.stop_publishing(
                    f'Driver did not start the test: {self.driver.last_rejection}')
                return
            command = DriveCommand()
            command.header.stamp = self.get_clock().now().to_msg()
            command.header.frame_id = self.vehicle.header.frame_id
            command.session_id = self.session
            command.gear = DriveCommand.GEAR_NEUTRAL
            command.speed_mps = 0.0
            command.steering_angle_rad = self.steering_angle_rad
            command.brake_percent = self.percent
            self.publisher.publish(command)
            if now - self.report_at >= 0.2:
                self.report_at = now
                self.get_logger().info(
                    f'request={self.percent:g}%, mode={self.vehicle.drive_mode}, '
                    f'speed={self.vehicle.speed_mps:.3f} m/s, '
                    f'brake_pressure={self.vehicle.brake_pressure_pa:.0f} Pa, '
                    f'driver_active={self.driver.drive_active}, '
                    f'mode_confirmed={self.driver.drive_mode_confirmed}')
        elif self.phase == 'observing':
            if (fresh and self.driver_at > self.stopped_at and
                    not self.driver.drive_active):
                self.observed_inactive = True
            # Default driver command timeout + invalidation burst fits in 1.5 s.
            # Feedback is still reported honestly if those settings were changed.
            if now - self.stopped_at >= 1.5:
                self.finish()


def main(args=None):
    """Run a single explicit percentage request with no automatic retries."""
    arguments = sys.argv[1:] if args is None else args
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--brake-percent', required=True, type=brake_percent)
    parser.add_argument('--duration', type=test_duration, default=1.0)
    parser.add_argument('--namespace', default='/withus')
    options = parser.parse_args(remove_ros_args(
        args=['brake_response_check'] + arguments)[1:])
    rclpy.init(args=arguments, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = BrakeResponseCheck(
            options.brake_percent, options.duration, options.namespace)
        try:
            while rclpy.ok() and not node.done:
                rclpy.spin_once(node, timeout_sec=0.05)
        except KeyboardInterrupt:
            node.stop_publishing('Interrupted; no further drive commands will be published.')
            while rclpy.ok() and not node.done:
                rclpy.spin_once(node, timeout_sec=0.05)
        return node.exit_code
    except (KeyboardInterrupt, ExternalShutdownException):
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())

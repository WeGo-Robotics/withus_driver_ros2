"""Show vehicle feedback and detect missing ROS status messages."""

import math
import time

from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from withus_msgs.msg import VehicleStatus


class StatusMonitor(Node):
    """Read vehicle feedback without depending on the CAN driver internals."""

    def __init__(self):
        """Create the subscription and a steady-clock reporting timer."""
        super().__init__('status_monitor', namespace='withus')

        self.report_period_sec = self._read_period('report_period_sec', 1.0)
        self.status_topic_timeout_sec = self._read_period(
            'status_topic_timeout_sec', 0.5
        )

        self.latest_status = None
        self.last_received_at = 0.0
        self.last_report_at = 0.0
        self.connection_state = None

        self.subscription = self.create_subscription(
            VehicleStatus, 'vehicle_status', self.on_status, 1
        )
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(
            0.1, self.on_timer, clock=self.steady_clock
        )
        self.get_logger().info(
            f'Monitoring {self.subscription.topic_name}'
        )

    def _read_period(self, name, default):
        descriptor = ParameterDescriptor(
            description='Seconds; must be finite and at least 0.1.',
            read_only=True,
        )
        value = self.declare_parameter(name, default, descriptor).value
        if not math.isfinite(value) or value < 0.1:
            raise ValueError(f'{name} must be finite and at least 0.1 seconds')
        return value

    def on_status(self, msg):
        """Keep only the latest message and its local arrival time."""
        self.latest_status = msg
        self.last_received_at = time.monotonic()

    def on_timer(self):
        """Check topic silence and periodically report fresh feedback groups."""
        now = time.monotonic()

        if self.latest_status is None:
            state = 'WAITING'
        elif now - self.last_received_at > self.status_topic_timeout_sec:
            state = 'NO_ROS_MESSAGES'
        else:
            state = 'RECEIVING'

        if state != self.connection_state:
            self.connection_state = state
            if state == 'RECEIVING':
                self.get_logger().info('ROS status reception active.')
            elif state == 'WAITING':
                self.get_logger().warning('Waiting for vehicle_status.')
            else:
                self.get_logger().warning(
                    'No ROS status messages: cached values are not current.'
                )

        if state != 'RECEIVING':
            return
        if now - self.last_report_at < self.report_period_sec:
            return
        self.last_report_at = now

        msg = self.latest_status
        gear = {
            VehicleStatus.GEAR_P: 'P',
            VehicleStatus.GEAR_D: 'D',
            VehicleStatus.GEAR_N: 'N',
            VehicleStatus.GEAR_R: 'R',
        }.get(
            msg.gear, f'UNKNOWN({msg.gear})'
        )
        mode = {
            VehicleStatus.DRIVE_MODE_REMOTE: 'REMOTE',
            VehicleStatus.DRIVE_MODE_AD: 'AD',
            VehicleStatus.DRIVE_MODE_PARALLEL: 'PARALLEL',
            VehicleStatus.DRIVE_MODE_SEMI_AUTONOMOUS: 'SEMI_AUTO',
        }.get(
            msg.drive_mode, f'UNKNOWN({msg.drive_mode})'
        )
        faults = [
            name for name in ('motor', 'remote', 'dbs', 'bms', 'eps')
            if getattr(msg, f'{name}_fault')
        ]

        diagnosis = self.describe_group(
            msg, '301',
            f'voltage={msg.battery_voltage_v:.1f} V, '
            f'e_stop={msg.emergency_stop_pressed}, '
            f'faults={",".join(faults) if faults else "none"}'
        )
        operation = self.describe_group(
            msg, '303',
            f'gear={gear}, mode={mode}, hold={msg.brake_hold_active}, '
            f'requested_speed={msg.requested_speed_mps:.2f} m/s, '
            f'SOC={msg.battery_soc * 100.0:.1f}%'
        )
        motion = self.describe_group(
            msg, '304',
            f'speed={msg.speed_mps:.2f} m/s, '
            f'steering={msg.steering_angle_rad:.3f} rad, '
            f'brake={msg.brake_pressure_pa:.0f} Pa'
        )
        self.get_logger().info(
            f'\n{diagnosis}\n{operation}\n{motion}'
        )

    @staticmethod
    def describe_group(msg, can_id, values):
        """Hide values that have never arrived or whose feedback is stale."""
        prefix = f'status_{can_id}'
        if not getattr(msg, f'{prefix}_received'):
            return f'{can_id}: NOT_RECEIVED'

        age = getattr(msg, f'{prefix}_age_sec')
        if not getattr(msg, f'{prefix}_valid'):
            return f'{can_id}: STALE (age at publish={age:.3f} s)'

        return f'{can_id}: VALID (age at publish={age:.3f} s) | {values}'


def main(args=None):
    """Run the monitor until shutdown is requested."""
    rclpy.init(args=args)
    node = None
    try:
        node = StatusMonitor()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

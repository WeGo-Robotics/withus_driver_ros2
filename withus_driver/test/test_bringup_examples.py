#!/usr/bin/env python3
"""Verify the installed driver/client launch split on an existing isolated vcan.

The test starts its own driver and clients. It accepts no physical CAN link and
models no physical stopping performance. Source install/setup.bash before use.
"""

import argparse
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

os.environ.setdefault('ROS_DOMAIN_ID', '186')
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ.setdefault('ROS_LOG_DIR', tempfile.mkdtemp(prefix='withus-launch-test-'))

import rclpy
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from withus_msgs.msg import DriverStatus

from can_test_support import require_vcan
from test_hardware_drive import ACTUATORS, DrivePeer


class LaunchCheck:
    def __init__(self, interface):
        self.peer = DrivePeer(interface).start()
        self.peer.healthy()
        self.node = rclpy.create_node('launch_check')
        self.status = None
        self.sub = self.node.create_subscription(
            DriverStatus, '/withus/driver_status', self.on_status, 10)
        self.processes = []
        self.checks = 0

    def on_status(self, message):
        self.status = message

    def check(self, condition, description):
        if not condition:
            raise AssertionError(description)
        self.checks += 1
        print('PASS ' + description, flush=True)

    def wait(self, predicate, description, timeout=6.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if self.peer.error:
                raise RuntimeError(str(self.peer.error))
            rclpy.spin_once(self.node, timeout_sec=0.01)
            if predicate():
                self.check(True, description)
                return
        self.check(False, 'Timed out: ' + description)

    def start(self, package, launch, *arguments):
        fd, path = tempfile.mkstemp(prefix='withus-launch-', suffix='.log')
        log = os.fdopen(fd, 'w')
        process = subprocess.Popen(['ros2', 'launch', package, launch, *arguments],
                                   stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        self.processes.append((process, log, Path(path)))
        return process, Path(path)

    def stop(self, process):
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
        self.check(process.wait(timeout=6.0) == 0, 'owned launch exits cleanly')

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
        self.node.destroy_node()
        self.peer.close()


def run(h, interface):
    bringup_share = Path(get_package_share_directory('withus_bringup'))
    h.check({p.name for p in (bringup_share / 'launch').glob('*.launch.py')} ==
            {'withus_bringup.launch.py'}, 'only the standard driver launch is installed')
    example_share = Path(get_package_share_directory('withus_examples'))
    h.check({p.name for p in (example_share / 'launch').glob('*.launch.py')} ==
            {'status_monitor.launch.py', 'brake_response_check.launch.py'},
            'each client example has its own installed launch')
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
    h.wait(lambda: h.status is not None and h.status.drive_ready, 'default bringup becomes ready')
    h.check(h.status.control_profile == 'hardware_drive' and h.status.enable_tx and
            h.status.control_requested, 'default profile and TX are hardware_drive and enabled')
    h.check(not h.status.drive_active and {f.can_id for f in h.peer.history()} == {0x501},
            'driver startup sends only 501 until a client command arrives')

    monitor, monitor_log = h.start('withus_examples', 'status_monitor.launch.py',
                                   'report_period_sec:=0.1')
    h.wait(lambda: 'ROS status reception active.' in monitor_log.read_text(),
           'status client launch receives the separately running driver')
    h.check(sum(name == 'withus_driver' and namespace == '/withus'
                for name, namespace in h.node.get_node_names_and_namespaces()) == 1,
            'client launch does not start another driver')
    h.check({f.can_id for f in h.peer.history()} == {0x501}, 'status client adds no actuator commands')
    h.stop(monitor)
    h.check(driver.poll() is None, 'stopping the status client leaves the driver running')

    begin = time.monotonic()
    client, client_log = h.start('withus_examples', 'brake_response_check.launch.py',
                                'brake_percent:=13', 'duration:=0.4')
    h.wait(lambda: client.poll() is not None, 'brake client finishes its bounded run', 9.0)
    output = client_log.read_text()
    h.check(client.returncode == 0 and 'Driver active observed=True' in output and
            'active=false after publishing stopped=True' in output and
            'process has finished cleanly' in output,
            'brake client observes activation and later invalidation with a clean exit')
    frames = [f for f in h.peer.history(begin) if f.can_id in ACTUATORS and f.data[0] & 1]
    h.check({f.can_id for f in frames} == set(ACTUATORS), 'brake example sends a whole DriveCommand')
    h.check(all(f.data[1] == 13 for f in frames if f.can_id == 0x503),
            'brake_percent launch argument reaches CAN unchanged')
    h.check(all(f.data[3] == 2 and int.from_bytes(f.data[6:8], 'little') == 0
                for f in frames if f.can_id == 0x504), 'stationary example keeps Neutral and zero speed')
    h.check(all(int.from_bytes(f.data[4:6], 'little') == 300
                for f in frames if f.can_id == 0x502), 'stationary example holds the observed steering')
    h.wait(lambda: h.status and not h.status.drive_active and h.status.drive_ready,
           'driver is ready for a new client after bounded invalidation')
    h.check(driver.poll() is None and h.status.control_requested,
            'ending the brake client leaves the driver and continuous 501 running')
    h.stop(driver)

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

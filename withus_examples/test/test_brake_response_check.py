"""Exercise client command content and one-shot behavior without CAN or DDS."""

import argparse
from types import SimpleNamespace

import pytest
from withus_msgs.msg import DriverStatus, VehicleStatus

from withus_examples import brake_response_check as brake


def vehicle_status():
    """Build stationary neutral feedback with a nonzero steering observation."""
    message = VehicleStatus()
    for can_id in ('301', '303', '304', '10a'):
        setattr(message, f'status_{can_id}_valid', True)
    message.gear = VehicleStatus.GEAR_N
    message.drive_mode = VehicleStatus.DRIVE_MODE_SEMI_AUTONOMOUS
    message.remote_a_raw = 1
    message.remote_a_selected = True
    message.steering_angle_rad = 0.12
    return message


class ClientHarness(brake.BrakeResponseCheck):
    """Run the production timer with in-memory streams and publication."""

    def __init__(self, clock):
        """Supply ROS boundary fakes without initializing a ROS node."""
        self.clock = clock
        self.percent = 10.0
        self.duration = 1.0
        self.driver = DriverStatus()
        self.driver.control_profile = 'hardware_drive'
        self.driver.drive_ready = True
        self.driver.control_requested = True
        self.driver.can_connected = True
        self.driver.enable_tx = True
        self.driver.session_id = 11
        self.vehicle = vehicle_status()
        self.driver_at = clock[0]
        self.vehicle_at = clock[0]
        self.phase = 'waiting'
        self.created_at = clock[0]
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
        self.published = []
        self.logs = []
        self.publisher = SimpleNamespace(
            get_subscription_count=lambda: 1, publish=self.published.append)

    def get_clock(self):
        """Return a positive ROS timestamp independently of monotonic time."""
        stamp = self.vehicle.header.stamp
        stamp.sec = int(self.clock[0])
        return SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=int(self.clock[0] * 1e9),
                                        to_msg=lambda: stamp))

    def get_logger(self):
        """Capture informational and rejection messages."""
        return SimpleNamespace(info=self.logs.append, warning=self.logs.append)

    def advance(self, elapsed, fresh=True):
        """Advance a test tick, optionally delivering new status messages."""
        self.clock[0] += elapsed
        if fresh:
            self.driver_at = self.clock[0]
            self.vehicle_at = self.clock[0]
        self.on_timer()


@pytest.fixture
def client(monkeypatch):
    """Use an explicit monotonic clock without sleeps or actual ROS entities."""
    clock = [100.0]
    monkeypatch.setattr(brake.time, 'monotonic', lambda: clock[0])
    return ClientHarness(clock)


@pytest.mark.parametrize('value', ['nan', 'inf', '-1', '101', 'text'])
def test_reject_bad_brake_percent(value):
    """Do not replace invalid caller input with a default brake amount."""
    with pytest.raises(argparse.ArgumentTypeError):
        brake.brake_percent(value)


@pytest.mark.parametrize('value', ['nan', 'inf', '0', '-1', '1.51', 'text'])
def test_reject_bad_duration(value):
    """Keep each client run bounded and reject nonfinite durations."""
    with pytest.raises(argparse.ArgumentTypeError):
        brake.test_duration(value)


@pytest.mark.parametrize(('field', 'value', 'reason'), [
    ('status_301_valid', False, 'stale'),
    ('status_303_valid', False, 'stale'),
    ('status_304_valid', False, 'stale'),
    ('status_10a_valid', False, 'stale'),
    ('remote_a_raw', 0, 'A switch'),
    ('remote_a_selected', False, 'A switch'),
    ('gear', VehicleStatus.GEAR_D, 'Neutral'),
    ('speed_mps', 0.06, 'speed'),
    ('speed_mps', -0.06, 'speed'),
    ('speed_mps', float('nan'), 'speed'),
    ('steering_angle_rad', 0.31, 'steering'),
    ('steering_angle_rad', float('nan'), 'steering'),
    ('drive_mode', VehicleStatus.DRIVE_MODE_REMOTE, 'mode'),
    ('emergency_stop_pressed', True, 'blocking'),
    ('front_bumper_triggered', True, 'blocking'),
    ('eps_fault', True, 'blocking'),
])
def test_stationary_feedback_requirements(field, value, reason):
    """Prevent launch readiness from bypassing vehicle-side requirements."""
    message = vehicle_status()
    setattr(message, field, value)
    assert reason in brake.stationary_problem(message)


def test_preserve_initial_steering_and_explicit_client_brake(client):
    """Publish whole zero-speed Neutral commands without steering recentering."""
    client.on_timer()
    assert len(client.published) == 1
    command = client.published[0]
    assert command.session_id == 11
    assert command.gear == command.GEAR_NEUTRAL
    assert command.speed_mps == 0.0
    assert command.steering_angle_rad == pytest.approx(0.12)
    assert command.brake_percent == 10.0
    client.vehicle.steering_angle_rad = 0.15
    client.driver.drive_active = True
    client.advance(0.02)
    assert client.published[-1].steering_angle_rad == pytest.approx(0.12)


def test_do_not_take_over_active_client(client):
    """Refuse to start while an existing drive target is active."""
    client.driver.drive_active = True
    client.on_timer()
    assert client.done
    assert not client.published
    assert 'already active' in client.stop_reason


def test_out_of_range_steering_never_centers(client):
    """Refuse an angle outside the client limit without publishing a target."""
    client.vehicle.steering_angle_rad = 0.4
    client.on_timer()
    assert client.done
    assert not client.published
    assert 'steering' in client.stop_reason


def test_wrong_profile_never_publishes(client):
    """Keep this stationary hardware client separate from the virtual policy."""
    client.driver.control_profile = 'vcan_reference'
    client.on_timer()
    assert client.done
    assert not client.published


def test_session_change_never_restarts(client):
    """Discard the client run instead of following a new driver session."""
    client.on_timer()
    client.driver.session_id = 12
    client.advance(0.02)
    assert client.phase == 'observing'
    assert client.session == 11
    assert len(client.published) == 1
    client.advance(0.02)
    assert len(client.published) == 1


def test_stale_ros_messages_never_restart(client):
    """Stop publication even when cached driver valid flags remain true."""
    client.on_timer()
    client.advance(0.26, fresh=False)
    assert client.phase == 'observing'
    assert 'stale' in client.stop_reason
    client.advance(0.02)
    assert len(client.published) == 1


@pytest.mark.parametrize(('field', 'value'), [
    ('gear', VehicleStatus.GEAR_D),
    ('speed_mps', 0.1),
    ('remote_a_selected', False),
    ('status_304_valid', False),
    ('rear_bumper_triggered', True),
])
def test_stop_when_stationary_conditions_are_lost(client, field, value):
    """Never publish another command after locally observing a lost condition."""
    client.on_timer()
    setattr(client.vehicle, field, value)
    client.advance(0.02)
    assert client.phase == 'observing'
    assert len(client.published) == 1


def test_driver_rejection_is_reported_without_retry_session(client):
    """Expose a stricter driver limit instead of modifying or clamping a target."""
    client.on_timer()
    client.driver.last_rejection = 'Steering command exceeds configured range.'
    client.advance(0.31)
    assert client.phase == 'observing'
    assert 'Steering command exceeds' in client.stop_reason
    assert len(client.published) == 1


def test_completed_duration_waits_for_fresh_inactive_status(client):
    """Bound publication and observe driver inactivity without claiming release."""
    client.on_timer()
    client.driver.drive_active = True
    client.advance(0.02)
    client.advance(0.99)
    count = len(client.published)
    assert client.phase == 'observing'
    assert client.saw_active
    client.driver.drive_active = False
    client.advance(0.02)
    client.advance(1.49)
    assert client.done
    assert client.exit_code == 0
    assert client.observed_inactive
    assert len(client.published) == count

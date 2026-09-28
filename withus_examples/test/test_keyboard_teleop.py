"""Exercise keyboard intent, feedback handshakes, and terminal escape parsing."""

import errno
import os
import termios
from types import SimpleNamespace as NS

import pytest

from withus_examples.keyboard_teleop import (
    BLOCKING_FIELDS, KeyParser, Limits, Target, TeleopController, TerminalInput,
)


def stamp(nanoseconds):
    return NS(sec=nanoseconds // 1_000_000_000,
              nanosec=nanoseconds % 1_000_000_000)


class Rig:
    """Provide fresh independent driver and CAN-derived vehicle snapshots."""

    def __init__(self, **options):
        self.client = TeleopController(**options)
        self.client.set_limits(Limits(1.0, 0.5, 0.3, 0.2, 0.05))
        self.now = 10.0
        self.ros_ns = 1_000_000_000_000
        self.driver = NS(
            header=NS(stamp=stamp(self.ros_ns)), control_profile='hardware_drive',
            can_connected=True, enable_tx=True, control_requested=True,
            session_id=7, drive_ready=True, drive_active=False,
            drive_mode_confirmed=False, drive_gear_pending=False,
            drive_reason='', last_rejection='')
        self.vehicle = NS(
            header=NS(stamp=stamp(self.ros_ns)), gear=2, speed_mps=0.0,
            steering_angle_rad=0.0, drive_mode=3, ad_signal_received=True,
            remote_a_raw=1, remote_a_selected=True,
            status_303_stamp=stamp(self.ros_ns))
        for name in BLOCKING_FIELDS:
            setattr(self.vehicle, name, False)
        for can_id in ('301', '303', '304', '10a'):
            setattr(self.vehicle, f'status_{can_id}_valid', True)
        self.refresh()
        self.client.tick(self.now, self.ros_ns)

    def refresh(self):
        self.driver.header.stamp = stamp(self.ros_ns)
        self.vehicle.header.stamp = stamp(self.ros_ns)
        self.vehicle.status_303_stamp = stamp(self.ros_ns)
        self.client.on_driver(self.driver, self.now)
        self.client.on_vehicle(self.vehicle, self.now)

    def advance(self, seconds=0.02, fresh=True):
        self.now += seconds
        self.ros_ns += round(seconds * 1_000_000_000)
        if fresh:
            self.refresh()
        return self.client.tick(self.now, self.ros_ns)

    def key(self, key):
        self.client.handle_key(key, self.now, self.ros_ns)
        return self.client.target

    def arm(self, gear=1):
        self.vehicle.gear = gear
        self.advance(0.21)
        self.key('s')
        self.driver.drive_active = True
        self.driver.drive_mode_confirmed = True
        self.vehicle.drive_mode = 1
        self.advance()
        assert self.client.phase == 'holding'


def test_launch_and_motion_keys_do_not_start_a_session():
    rig = Rig()
    assert rig.advance() is None
    for key in 'wadqz':
        rig.key(key)
        assert rig.client.target is None
    assert rig.client.session is None


def test_bootstrap_preserves_current_gear_and_requires_ad_before_motion():
    rig = Rig()
    rig.vehicle.gear = 3
    assert rig.key('s') == Target(3)
    rig.key('w')
    assert rig.client.target == Target(3)
    rig.driver.drive_active = True
    rig.advance(0.21)
    assert rig.client.phase == 'preparing'
    rig.driver.drive_mode_confirmed = True
    rig.vehicle.drive_mode = 1
    rig.advance()
    assert rig.client.target == Target(3)
    rig.key('w')
    assert rig.client.target == Target(3, 0.1, 0.0, 0.0)


def test_gear_key_bootstraps_zero_then_waits_for_new_ack_and_new_motion_key():
    rig = Rig()
    rig.advance(0.21)
    assert rig.key('2') == Target(2)  # First command remains in observed N.
    rig.driver.drive_active = True
    rig.driver.drive_mode_confirmed = True
    rig.vehicle.drive_mode = 1
    assert rig.advance() == Target(1)
    assert rig.client.phase == 'gear'
    rig.key('w')
    assert rig.client.target == Target(1)
    rig.vehicle.gear = 1
    # A matching gear in an old snapshot cannot acknowledge the request.
    rig.client.tick(rig.now, rig.ros_ns)
    assert rig.client.phase == 'gear'
    rig.driver.drive_gear_pending = True
    rig.advance()
    assert rig.client.phase == 'gear'
    rig.driver.drive_gear_pending = False
    rig.advance()
    assert rig.client.phase == 'holding'
    assert rig.client.target == Target(1)
    rig.key('w')
    assert rig.client.target.speed == 0.1


@pytest.mark.parametrize('key,angle', [('w', 0.0), ('a', 0.3), ('d', -0.3)])
def test_motion_keys_and_explicit_s_stop(key, angle):
    rig = Rig()
    rig.arm()
    assert rig.key(key) == Target(1, 0.1, angle, 0.0)
    assert rig.key('s') == Target(1, 0.0, 0.0, 80.0)
    assert rig.client.phase == 'holding'


def test_configurable_steering_sign_and_driver_limits():
    rig = Rig(left_sign=-1)
    rig.client.set_limits(Limits(0.15, 0.08, 0.12, 0.2, 0.05))
    rig.arm()
    rig.key('q')
    assert rig.client.selected_speed == 0.15
    assert rig.key('a') == Target(1, 0.15, -0.12, 0.0)
    assert rig.key('d') == Target(1, 0.15, 0.12, 0.0)
    rig.key('s')
    rig.key('4')
    rig.vehicle.gear = 3
    rig.advance()
    assert rig.key('w') == Target(3, 0.08, 0.0, 0.0)


def test_speed_adjustment_clamps_without_starting_motion():
    rig = Rig()
    rig.arm()
    for _ in range(20):
        rig.key('q')
    assert rig.client.selected_speed == 1.0
    assert rig.client.target == Target(1)
    rig.key('w')
    assert rig.client.target.speed == 1.0
    rig.key('z')
    assert rig.client.target.speed == 0.9
    for _ in range(20):
        rig.key('z')
    assert rig.client.selected_speed == 0.0
    assert rig.client.target == Target(1, 0.0, 0.0, 0.0)
    rig.key('q')
    assert rig.client.target == Target(1, 0.0, 0.0, 0.0)
    rig.key('w')
    assert rig.client.target.speed == 0.1


def test_input_timeout_stops_publication_without_braking_or_replaying():
    rig = Rig(key_timeout=1.0)
    rig.arm()
    rig.key('a')
    rig.advance(0.6)
    assert rig.client.target.speed == 0.1
    rig.key('q')  # Speed adjustment counts as active user input.
    rig.advance(0.6)
    assert rig.client.target.speed == 0.2
    assert rig.advance(0.41) is None
    assert 'no brake was requested' in rig.client.notice
    assert rig.advance(1.0) is None
    rig.key('w')
    assert rig.client.target is None
    rig.driver.drive_active = False
    rig.driver.drive_mode_confirmed = False
    rig.driver.session_id += 1
    rig.advance()
    assert rig.key('s') == Target(1)


def test_moving_gear_is_rejected_and_s_preserves_pending_gear():
    rig = Rig()
    rig.arm()
    rig.key('w')
    rig.vehicle.speed_mps = 0.1
    rig.advance()
    rig.key('4')
    assert rig.client.target.gear == 1
    assert 'refused' in rig.client.notice
    rig.key('s')
    rig.vehicle.speed_mps = 0.0
    rig.advance()
    rig.key('4')
    assert rig.client.target.gear == 1
    rig.advance(0.21)
    assert rig.key('4') == Target(3)
    assert rig.key('s') == Target(3)
    rig.key('3')
    assert rig.client.target == Target(3)


@pytest.mark.parametrize('gear', [0, 2])
def test_park_and_neutral_never_receive_motion_targets(gear):
    rig = Rig()
    rig.arm(gear)
    for key in 'wad':
        assert rig.key(key) == Target(gear)


@pytest.mark.parametrize('fault', ['remote', 'session', 'inactive', 'can',
                                   'mode', 'stale', 'delayed', 'blocked'])
def test_lost_conditions_discard_targets_and_require_manual_rearm(fault):
    rig = Rig()
    rig.arm()
    rig.key('w')
    if fault == 'remote':
        rig.vehicle.remote_a_selected = False
    elif fault == 'session':
        rig.driver.session_id += 1
    elif fault == 'inactive':
        rig.driver.drive_active = False
    elif fault == 'can':
        rig.vehicle.status_304_valid = False
    elif fault == 'mode':
        rig.vehicle.drive_mode = 3
    elif fault == 'blocked':
        rig.vehicle.eps_fault = True
    elif fault == 'delayed':
        rig.driver.header.stamp = stamp(rig.ros_ns - 300_000_000)
    if fault == 'stale':
        rig.advance(0.3, fresh=False)
    else:
        rig.client.tick(rig.now, rig.ros_ns)
    assert rig.client.target is None
    assert rig.client.session is None
    rig.vehicle.remote_a_selected = True
    rig.vehicle.status_304_valid = True
    rig.vehicle.eps_fault = False
    rig.vehicle.drive_mode = 3
    rig.driver.drive_active = False
    rig.driver.drive_mode_confirmed = False
    rig.advance(0.21)
    assert rig.client.target is None
    rig.key('w')
    assert rig.client.target is None
    assert rig.key('s') == Target(1)


def test_a_second_client_cannot_claim_an_already_active_session():
    rig = Rig()
    rig.driver.drive_active = True
    rig.key('s')
    assert rig.client.session is None
    assert 'already active' in rig.client.notice


def test_clock_rollback_discards_motion_and_invalid_limits_fail_closed():
    rig = Rig()
    rig.arm()
    rig.key('w')
    assert rig.client.tick(rig.now, rig.ros_ns - 1) is None
    with pytest.raises(ValueError):
        Limits(1, 0.5, float('nan'), 0.2, 0.05)
    for timeout in (0, -1, float('nan')):
        with pytest.raises(ValueError):
            TeleopController(key_timeout=timeout)
    rig.client.limits = None
    rig.key('s')
    assert rig.client.target is None


def test_parameter_types_and_units():
    values = [NS(type=3, double_value=x) for x in (1.0, 0.5, 0.3)]
    values += [NS(type=2, integer_value=200), NS(type=3, double_value=0.05)]
    assert Limits.from_parameters(values) == Limits(1, 0.5, 0.3, 0.2, 0.05)
    values[0] = NS(type=1)
    with pytest.raises(ValueError):
        Limits.from_parameters(values)
    with pytest.raises(ValueError):
        Limits.from_parameters([])


def test_terminal_parser_discards_arrows_alt_keys_and_pasted_commands():
    parser = KeyParser()
    assert parser.feed(b'w\x1b[A\x1b[B\x1b[C\x1b[D\x1bOAs') == ['w', 's']
    assert parser.feed(b'\x1bw') == []
    assert parser.feed(b'\x1b[200~wasd1234qz') == []
    assert parser.feed(b'wasd\x1b[20') == []
    assert parser.feed(b'1~2') == ['2']
    assert parser.feed(b'\x1b[') == []
    assert parser.feed(b'3~q') == ['q']
    assert parser.feed(b'\x03') == ['\x03']


@pytest.mark.parametrize('key', ['s', '2'])
def test_inactivity_also_ends_stop_and_gear_preparation_commands(key):
    rig = Rig()
    rig.arm(2)
    rig.key(key)
    assert rig.client.target is not None
    assert rig.advance(1.01) is None
    assert rig.client.queued_gear is None


def test_stationary_steering_requires_fresh_motion_key_to_accelerate():
    rig = Rig()
    rig.arm()
    rig.key('z')
    assert rig.key('a') == Target(1, 0.0, 0.3, 80.0)
    rig.key('q')
    assert rig.client.selected_speed == 0.1
    assert rig.client.target == Target(1, 0.0, 0.3, 80.0)
    assert rig.key('w') == Target(1, 0.1, 0.0, 0.0)


def test_old_driver_snapshot_cannot_acknowledge_new_vehicle_gear():
    rig = Rig()
    rig.arm()
    rig.key('4')
    requested = rig.client.gear_request_ns
    rig.driver.header.stamp = stamp(requested + 1)
    rig.vehicle.gear = 3
    rig.vehicle.status_303_stamp = stamp(requested + 2)
    rig.client.tick(rig.now, rig.ros_ns)
    assert rig.client.phase == 'gear'
    rig.driver.header.stamp = stamp(requested + 3)
    rig.client.tick(rig.now, rig.ros_ns)
    assert rig.client.phase == 'holding'
    assert rig.client.target.speed == 0.0


def test_terminal_hangup_does_not_prevent_remaining_cleanup():
    master, slave = os.openpty()
    terminal = TerminalInput.__new__(TerminalInput)
    terminal.fd = slave
    terminal.saved = termios.tcgetattr(slave)
    os.close(master)  # tcsetattr(slave) now raises EIO on Linux.
    terminal.close()
    assert terminal.fd is None
    with pytest.raises(OSError) as error:
        os.fstat(slave)
    assert error.value.errno == errno.EBADF
    terminal.close()  # Repeated finally/cleanup calls are harmless.

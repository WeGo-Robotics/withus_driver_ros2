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


@pytest.mark.parametrize('key,angle', [('a', 0.3), ('d', -0.3)])
def test_steering_only_keeps_stopped_speed_brake_and_gear(key, angle):
    rig = Rig()
    rig.arm()
    assert rig.client.selected_speed == 0.1
    assert rig.key(key) == Target(1, 0.0, angle, 50.0)
    assert rig.key('s') == Target(1, 0.0, 0.0, 50.0)
    assert rig.client.phase == 'holding'


@pytest.mark.parametrize('gear', [1, 3])
def test_speed_and_steering_change_independently_with_same_gear(gear):
    rig = Rig()
    rig.arm(gear)
    assert rig.key('a') == Target(gear, 0.0, 0.3, 50.0)
    assert rig.key('w') == Target(gear, 0.1, 0.3, 0.0)
    assert rig.key('q') == Target(gear, 0.2, 0.3, 0.0)
    assert rig.key('d') == Target(gear, 0.2, -0.3, 0.0)
    assert rig.key('w') == Target(gear, 0.2, -0.3, 0.0)
    assert rig.key('z') == Target(gear, 0.1, -0.3, 0.0)
    assert rig.key('s') == Target(gear, 0.0, 0.0, 50.0)


def test_configurable_steering_sign_and_driver_limits():
    rig = Rig(left_sign=-1)
    rig.client.set_limits(Limits(0.15, 0.08, 0.12, 0.2, 0.05))
    rig.arm()
    rig.key('q')
    assert rig.client.selected_speed == 0.15
    assert rig.key('a') == Target(1, 0.0, -0.12, 50.0)
    assert rig.key('w') == Target(1, 0.15, -0.12, 0.0)
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
    rig.key('w')
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
    assert rig.key('w') == Target(gear)
    assert rig.key('a') == Target(gear, 0.0, 0.3, 50.0)
    assert rig.key('d') == Target(gear, 0.0, -0.3, 50.0)
    assert rig.key('w') == Target(gear, 0.0, -0.3, 50.0)


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
    assert rig.key('a') == Target(1, 0.0, 0.3, 50.0)
    rig.key('q')
    assert rig.client.selected_speed == 0.2
    assert rig.client.target == Target(1, 0.0, 0.3, 50.0)
    assert rig.key('w') == Target(1, 0.2, 0.3, 0.0)


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


@pytest.mark.parametrize('key,gear', [('1', 0), ('2', 1), ('4', 3)])
def test_one_gear_press_survives_ad_confirmation_before_local_stopped_dwell(key, gear):
    rig = Rig()
    # Driver is already ready, while this newly started client has less than
    # 200 ms of local feedback. Fast AD confirmation must not erase the key.
    assert rig.key(key) == Target(2)
    rig.driver.drive_active = True
    rig.driver.drive_mode_confirmed = True
    rig.vehicle.drive_mode = 1
    assert rig.advance(0.02) == Target(2)
    assert rig.client.phase == 'waiting_gear'
    assert rig.client.queued_gear == gear
    rig.key('w')
    assert rig.client.target == Target(2)
    assert rig.advance(0.19) == Target(gear)
    assert rig.client.phase == 'gear'
    rig.vehicle.gear = gear
    assert rig.advance() == Target(gear)
    assert rig.client.phase == 'holding'
    for key in 'adqzs':
        rig.key(key)
        assert rig.advance(0.1).gear == gear
        assert rig.client.target.speed == 0.0


def test_repeating_pending_gear_keeps_input_alive_without_restarting_ack_deadline():
    rig = Rig()
    rig.arm()
    rig.key('4')
    requested = rig.client.gear_request_ns
    rig.driver.drive_gear_pending = True
    rig.advance(0.6)
    rig.key('4')
    assert rig.advance(0.5) == Target(3)
    assert rig.client.gear_request_ns == requested
    rig.key('2')  # A different gear cannot replace an in-flight request.
    assert rig.client.target == Target(3)
    assert rig.advance(0.51) is None


def test_s_cancels_a_gear_queued_before_stationary_dwell():
    rig = Rig()
    rig.key('2')
    rig.driver.drive_active = True
    rig.driver.drive_mode_confirmed = True
    rig.vehicle.drive_mode = 1
    rig.advance()
    assert rig.client.phase == 'waiting_gear'
    assert rig.key('s') == Target(2)
    assert rig.client.queued_gear is None
    assert rig.advance(0.21) == Target(2)
    assert rig.client.phase == 'holding'


def test_idle_remembers_confirmed_gear_but_only_explicit_s_can_restore_it():
    rig = Rig()
    rig.arm(1)
    rig.key('w')
    rig.key('d')
    assert rig.advance(1.01) is None
    assert rig.client.selected_gear == 1
    assert rig.client.session is None
    rig.driver.drive_active = False
    rig.driver.drive_mode_confirmed = False
    rig.driver.session_id += 1
    rig.vehicle.drive_mode = 3
    rig.vehicle.gear = 2  # Feedback may differ after command publication ends.
    assert rig.advance(0.21) is None
    assert rig.key('w') is None
    assert rig.key('a') is None
    assert rig.client.selected_gear == 1
    assert rig.key('s') == Target(2)  # Always start with the observed gear.
    assert rig.client.queued_gear == 1
    rig.driver.drive_active = True
    rig.driver.drive_mode_confirmed = True
    rig.vehicle.drive_mode = 1
    assert rig.advance() == Target(1)
    rig.vehicle.gear = 1
    assert rig.advance() == Target(1)
    assert rig.client.selected_gear == 1
    assert rig.client.phase == 'holding'
    assert rig.key('a') == Target(1, 0.0, 0.3, 50.0)
    assert rig.key('w') == Target(1, 0.1, 0.3, 0.0)


def test_unacknowledged_shift_is_not_remembered_on_idle():
    rig = Rig()
    rig.arm(1)
    rig.key('4')
    assert rig.client.target.gear == 3
    assert rig.client.selected_gear == 1
    assert rig.advance(1.01) is None
    assert rig.client.selected_gear == 1
    assert rig.client.queued_gear is None


@pytest.mark.parametrize('fault', ['remote', 'stale', 'blocked'])
def test_fault_while_idle_clears_remembered_gear(fault):
    rig = Rig()
    rig.arm(1)
    rig.advance(1.01)
    assert rig.client.selected_gear == 1
    rig.driver.drive_active = False
    rig.driver.drive_mode_confirmed = False
    rig.vehicle.drive_mode = 3
    rig.vehicle.gear = 2
    if fault == 'remote':
        rig.vehicle.remote_a_raw = 0
    elif fault == 'blocked':
        rig.vehicle.eps_fault = True
    if fault == 'stale':
        rig.advance(0.3, fresh=False)
    else:
        rig.advance()
    assert rig.client.selected_gear is None
    rig.vehicle.remote_a_raw = 1
    rig.vehicle.eps_fault = False
    rig.advance()
    assert rig.key('s') == Target(2)
    assert rig.client.queued_gear is None


@pytest.mark.parametrize('key,queued,remembered', [('4', 3, 1), ('3', None, 2)])
def test_new_numeric_key_overrides_remembered_gear(key, queued, remembered):
    rig = Rig()
    rig.arm(1)
    rig.advance(1.01)
    rig.driver.drive_active = False
    rig.driver.drive_mode_confirmed = False
    rig.vehicle.drive_mode = 3
    rig.vehicle.gear = 2
    rig.advance()
    assert rig.key('s') == Target(2)
    assert rig.client.queued_gear == 1
    rig.key(key)
    assert rig.client.queued_gear == queued
    assert rig.client.selected_gear == remembered


def test_repeated_s_cancels_remembered_gear_restoration_before_shift():
    rig = Rig()
    rig.arm(1)
    rig.advance(1.01)
    rig.driver.drive_active = False
    rig.driver.drive_mode_confirmed = False
    rig.vehicle.drive_mode = 3
    rig.vehicle.gear = 2
    rig.advance()
    rig.key('s')
    assert rig.client.queued_gear == 1
    rig.key('s')
    assert rig.client.queued_gear is None
    assert rig.client.selected_gear == 2
    rig.driver.drive_active = True
    rig.driver.drive_mode_confirmed = True
    rig.vehicle.drive_mode = 1
    assert rig.advance() == Target(2)


@pytest.mark.parametrize('brake', [0.0, 30.0, 30.5, 50.0, 100.0])
def test_configured_brake_applies_to_preparation_gear_stop_steering_and_rearm(brake):
    rig = Rig(stop_brake_percent=brake)
    rig.advance(0.21)
    assert rig.key('2') == Target(2, brake=brake)
    rig.driver.drive_active = True
    rig.driver.drive_mode_confirmed = True
    rig.vehicle.drive_mode = 1
    assert rig.advance() == Target(1, brake=brake)
    rig.vehicle.gear = 1
    assert rig.advance() == Target(1, brake=brake)
    assert rig.key('a') == Target(1, 0.0, 0.3, brake)
    assert rig.key('w') == Target(1, 0.1, 0.3, 0.0)
    assert rig.key('s') == Target(1, brake=brake)
    assert f'brake {brake:g}%' in rig.client.notice
    assert rig.key('d') == Target(1, 0.0, -0.3, brake)
    assert rig.key('4') == Target(3, brake=brake)
    assert rig.key('s') == Target(3, brake=brake)
    rig.vehicle.gear = 3
    assert rig.advance() == Target(3, brake=brake)
    assert rig.advance(1.01) is None
    rig.driver.drive_active = False
    rig.driver.drive_mode_confirmed = False
    rig.driver.session_id += 1
    rig.vehicle.drive_mode = 3
    rig.vehicle.gear = 2
    assert rig.advance(0.21) is None
    assert rig.key('s') == Target(2, brake=brake)
    rig.driver.drive_active = True
    rig.driver.drive_mode_confirmed = True
    rig.vehicle.drive_mode = 1
    assert rig.advance() == Target(3, brake=brake)
    rig.vehicle.gear = 3
    assert rig.advance() == Target(3, brake=brake)
    assert rig.key('a') == Target(3, 0.0, 0.3, brake)
    assert rig.key('w') == Target(3, 0.1, 0.3, 0.0)


def test_default_brake_is_50_and_zero_speed_w_preserves_configured_brake():
    rig = Rig()
    rig.arm()
    assert rig.client.stop_brake_percent == 50.0
    assert rig.client.target.brake == 50.0
    rig = Rig(initial_speed=0.0, stop_brake_percent=30.0)
    rig.arm()
    assert rig.key('w') == Target(1, brake=30.0)
    assert rig.key('a') == Target(1, 0.0, 0.3, 30.0)


@pytest.mark.parametrize('brake', [-0.1, 100.1, float('nan'), float('inf'), -float('inf')])
def test_invalid_stop_brake_is_rejected_before_commands_can_be_created(brake):
    with pytest.raises(ValueError, match='stop_brake_percent'):
        TeleopController(stop_brake_percent=brake)

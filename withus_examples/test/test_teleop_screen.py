"""Check display freshness, bounded repainting, and terminal cleanup."""

import errno
import fcntl
import os
import struct
import termios
from types import SimpleNamespace as NS

import pytest

from withus_examples.keyboard_teleop import Limits, Target, TeleopController
from withus_examples.teleop_screen import (
    ENTER_SCREEN, LEAVE_SCREEN, TerminalScreen, format_lines,
)


NOW = 10.0
ROS_NS = 1_000_000_000_000


def controller(**options):
    client = TeleopController(**options)
    stamp = NS(sec=1000, nanosec=0)
    client.set_limits(Limits(1.0, 0.5, 0.3, 0.2, 0.05))
    client.on_vehicle(NS(
        header=NS(stamp=stamp), gear=2, drive_mode=3,
        speed_mps=0.04, steering_angle_rad=0.01,
        remote_a_selected=True, remote_a_raw=1,
        status_303_valid=True, status_304_valid=True, status_10a_valid=True), NOW)
    client.on_driver(NS(
        header=NS(stamp=stamp), drive_ready=True, drive_mode_confirmed=False), NOW)
    return client


@pytest.mark.parametrize('options,brake', [
    ({}, 50), ({'stop_brake_percent': 30}, 30), ({'stop_brake_percent': 30.5}, 30.5),
])
def test_stop_help_matches_default_or_custom_controller_brake(options, brake):
    client = controller(**options)
    lines = '\n'.join(format_lines(client, NOW, ROS_NS))
    assert client.stop_brake_percent == brake
    assert f's: speed 0, brake {brake}%, steering center' in lines
    assert 'brake 80%' not in lines
    client.target = Target(2, brake=client.stop_brake_percent)
    client.session = 7
    active_lines = '\n'.join(format_lines(client, NOW, ROS_NS))
    assert f'Target steering: 0.000 rad | brake: {brake}%' in active_lines


def test_intent_and_observation_are_separate_without_mutating_controller():
    client = controller()
    client.selected_speed = 0.4
    client.selected_gear = 1
    client.session = 7
    client.target = Target(3, 0.2, -0.3, 0.0)
    client.phase = 'gear'
    client.last_command_key = NOW - 0.2
    before = vars(client).copy()
    lines = '\n'.join(format_lines(client, NOW, ROS_NS, '4'))
    assert 'Selected speed: 0.40 m/s | Remembered gear: D' in lines
    assert 'Actual gear: N | mode: 3 | A: ON' in lines
    assert 'Actual speed: 0.040 m/s | steering: 0.010 rad' in lines
    assert 'Command: ACTIVE | gear R | speed 0.20 m/s' in lines
    assert 'Target steering: -0.300 rad | brake: 0%' in lines
    assert 'Phase: gear | Pending gear: R' in lines
    assert 'Last key: 4 | Input: 0.8 / 1 s remaining' in lines
    assert vars(client) == before


@pytest.mark.parametrize('kind', ['arrival', 'old_stamp', 'future_stamp', 'zero_stamp'])
def test_stale_ros_data_hides_measurements_but_keeps_intent(kind):
    client = controller()
    client.selected_gear = 3
    if kind == 'arrival':
        client.vehicle_at = client.driver_at = NOW - 0.3
    elif kind == 'old_stamp':
        client.vehicle.header.stamp.sec -= 1
    elif kind == 'future_stamp':
        client.vehicle.header.stamp.nanosec = 50_000_001
    else:
        client.vehicle.header.stamp.sec = 0
    lines = '\n'.join(format_lines(client, NOW, ROS_NS))
    assert 'Remembered gear: R' in lines
    assert 'Actual gear: STALE | mode: STALE | A: STALE' in lines
    assert 'Actual speed: STALE m/s | steering: STALE rad' in lines
    assert 'Driver: STALE | AD: STALE' in lines
    assert 'Command: OFF (no commands published)' in lines


def test_independent_can_groups_do_not_hide_fresh_other_values():
    client = controller()
    client.vehicle.status_303_valid = False
    lines = '\n'.join(format_lines(client, NOW, ROS_NS))
    assert 'Actual gear: STALE | mode: STALE | A: ON' in lines
    assert 'Actual speed: 0.040 m/s' in lines
    client.vehicle.status_304_valid = False
    client.vehicle.status_10a_valid = False
    lines = '\n'.join(format_lines(client, NOW, ROS_NS))
    assert 'Actual gear: STALE | mode: STALE | A: STALE' in lines
    assert 'Actual speed: STALE m/s' in lines


def test_missing_feedback_and_limits_render_waiting_without_access_errors():
    lines = '\n'.join(format_lines(TeleopController(), NOW, ROS_NS))
    assert 'Remembered gear: --' in lines
    assert 'Actual gear: STALE' in lines
    assert 'Limits: waiting for driver parameters' in lines


@pytest.fixture
def terminal():
    master, slave = os.openpty()
    os.set_blocking(master, False)
    fcntl.ioctl(slave, termios.TIOCSWINSZ, struct.pack('HHHH', 24, 80, 0, 0))
    try:
        yield master, slave
    finally:
        os.close(master)
        os.close(slave)


def drain(fd):
    chunks = []
    while True:
        try:
            chunks.append(os.read(fd, 4096))
        except BlockingIOError:
            return b''.join(chunks)


def test_screen_repaints_fixed_rows_and_restores_owned_terminal(terminal):
    master, slave = terminal
    screen = TerminalScreen(slave)
    assert not os.get_blocking(slave)
    assert screen.render(controller(), NOW, ROS_NS, 'q')
    first = drain(master)
    assert first.startswith(ENTER_SCREEN)
    assert b'\x1b[?25l' in first
    assert b'\x1b[2;1H\x1b[2KSelected speed: 0.10' in first
    assert b'\x1b[3;1H\x1b[2KActual gear: N' in first
    assert b'\n' not in first and b'\r' not in first
    assert len(first) < 2048
    assert screen.render(controller(), NOW, ROS_NS)
    second = drain(master)
    assert b'\x1b[2;1H\x1b[2KSelected speed:' in second
    assert ENTER_SCREEN not in second
    screen.close()
    assert drain(master) == LEAVE_SCREEN
    assert os.get_blocking(slave)
    screen.close()
    assert screen.fd is None


def test_resize_reserves_last_row_and_column_and_sanitizes_text(terminal):
    master, slave = terminal
    screen = TerminalScreen(slave)
    drain(master)
    client = controller()
    client.notice = 'hello\x1b\n\r\t\x07world' + '한' * 100
    fcntl.ioctl(slave, termios.TIOCSWINSZ, struct.pack('HHHH', 18, 30, 0, 0))
    screen.render(client, NOW, ROS_NS)
    data = drain(master)
    assert b'\n' not in data and b'\r' not in data and b'\x07' not in data
    assert 'Notice: hello     world한한한'.encode() in data
    assert 'Notice: hello     world한한한한'.encode() not in data
    assert b'\x1b[18;' not in data
    assert b'\x1b[2J' in data
    fcntl.ioctl(slave, termios.TIOCSWINSZ, struct.pack('HHHH', 3, 20, 0, 0))
    screen.render(client, NOW, ROS_NS)
    data = drain(master)
    assert b'\x1b[3;' not in data
    assert b'Actual gear:' not in data
    assert b'Selected speed: 0.1' in data
    screen.close()


def test_zero_or_unavailable_terminal_size_uses_sensible_fallback(terminal, monkeypatch):
    master, slave = terminal
    screen = TerminalScreen(slave)
    fcntl.ioctl(slave, termios.TIOCSWINSZ, struct.pack('HHHH', 0, 0, 0, 0))
    screen.render(controller(), NOW, ROS_NS)
    assert b'Gear keys: 1=P  2=D  3=N  4=R' in drain(master)

    def unavailable(_fd):
        raise OSError(errno.ENOTTY, 'not a tty')

    monkeypatch.setattr(os, 'get_terminal_size', unavailable)
    screen.render(controller(), NOW, ROS_NS)
    assert b'Gear keys: 1=P  2=D  3=N  4=R' in drain(master)
    screen.close()


def test_backpressure_and_partial_writes_are_bounded_and_ordered(terminal, monkeypatch):
    master, slave = terminal
    screen = TerminalScreen(slave)
    drain(master)
    writes = []
    responses = [3, BlockingIOError(errno.EAGAIN, 'would block'), 99999, 99999]

    def write(_fd, data):
        writes.append(data)
        response = responses.pop(0)
        if isinstance(response, Exception):
            raise response
        return min(response, len(data))

    monkeypatch.setattr(os, 'write', write)
    assert not screen.render(controller(), NOW, ROS_NS)
    assert len(writes) == 1
    remaining = writes[0][3:]
    assert not screen.render(controller(), NOW, ROS_NS)
    assert len(writes) == 2 and writes[1] == remaining
    assert screen.render(controller(), NOW, ROS_NS)
    assert len(writes) == 4 and writes[2] == remaining
    responses.append(99999)
    screen.close()
    assert writes[-1] == LEAVE_SCREEN


@pytest.mark.parametrize('error_code', [errno.EIO, errno.ENXIO])
def test_hangup_during_render_and_close_is_tolerated(terminal, monkeypatch, error_code):
    master, slave = terminal
    screen = TerminalScreen(slave)
    drain(master)

    def disconnected(_fd, _data):
        raise OSError(error_code, 'terminal disappeared')

    monkeypatch.setattr(os, 'write', disconnected)
    assert not screen.render(controller(), NOW, ROS_NS)
    assert screen.disconnected
    screen.close()
    assert screen.fd is None
    assert os.get_blocking(slave)

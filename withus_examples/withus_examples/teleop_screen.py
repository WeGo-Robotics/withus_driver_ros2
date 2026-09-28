"""A fixed terminal view of keyboard intent and independently fresh feedback."""

import errno
import math
import os
import unicodedata


GEAR_NAMES = ('P', 'D', 'N', 'R')
ENTER_SCREEN = b'\x1b[?1049h\x1b[?25l\x1b[2J'
LEAVE_SCREEN = b'\x18\x1b[?25h\x1b[?1049l'


def _stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def _fresh(message, arrival, now, ros_ns):
    if message is None or ros_ns <= 0:
        return False
    stamp = _stamp_ns(message.header.stamp)
    return (stamp > 0 and 0.0 <= now - arrival <= 0.25 and
            -50_000_000 <= ros_ns - stamp <= 250_000_000)


def _gear(value):
    return GEAR_NAMES[value] if value in range(4) else '--'


def _number(value, digits=2):
    if not math.isfinite(value):
        return '--'
    return f'{value:g}' if digits is None else f'{value:.{digits}f}'


def format_lines(controller, now, ros_ns, last_key='-'):
    """Describe intent and feedback without advancing the controller's state."""
    vehicle, driver = controller.vehicle, controller.driver
    vehicle_fresh = _fresh(vehicle, controller.vehicle_at, now, ros_ns)
    driver_fresh = _fresh(driver, controller.driver_at, now, ros_ns)
    gear_fresh = vehicle_fresh and vehicle.status_303_valid
    motion_fresh = vehicle_fresh and vehicle.status_304_valid
    a_fresh = vehicle_fresh and vehicle.status_10a_valid
    actual_gear = _gear(vehicle.gear) if gear_fresh else 'STALE'
    mode = str(vehicle.drive_mode) if gear_fresh else 'STALE'
    a_switch = ('ON' if vehicle.remote_a_selected and vehicle.remote_a_raw == 1
                else 'OFF') if a_fresh else 'STALE'
    actual_speed = _number(vehicle.speed_mps, 3) if motion_fresh else 'STALE'
    actual_steer = _number(vehicle.steering_angle_rad, 3) if motion_fresh else 'STALE'
    active = controller.target is not None and controller.session is not None
    if active:
        target = controller.target
        command = (f'Command: ACTIVE | gear {_gear(target.gear)} | '
                   f'speed {_number(target.speed)} m/s')
        actuator = (f'Target steering: {_number(target.steering, 3)} rad | '
                    f'brake: {_number(target.brake, None)}%')
        remaining = max(0.0, controller.key_timeout - (now - controller.last_command_key))
        timeout = f'{remaining:.1f} / {controller.key_timeout:g} s remaining'
    else:
        command = 'Command: OFF (no commands published)'
        actuator = 'Target steering: -- | brake: --'
        timeout = f'inactive (limit {controller.key_timeout:g} s)'
    if driver_fresh:
        ready = 'ready' if driver.drive_ready else 'not ready'
        confirmed = 'confirmed' if driver.drive_mode_confirmed else 'not confirmed'
        driver_text = f'Driver: {ready} | AD: {confirmed}'
    else:
        driver_text = 'Driver: STALE | AD: STALE'
    pending_gear = controller.queued_gear
    if controller.phase == 'gear' and controller.target is not None:
        pending_gear = controller.target.gear
    if controller.limits is None:
        limits = 'Limits: waiting for driver parameters'
    else:
        limits = (f'Limits: forward {controller.limits.forward:.2f} | '
                  f'reverse {controller.limits.reverse:.2f} m/s | '
                  f'steer +/-{controller.limits.steering:.3f} rad')
    return [
        'Withus keyboard client | hardware_drive',
        f'Selected speed: {controller.selected_speed:.2f} m/s | '
        f'Remembered gear: {_gear(controller.selected_gear)}',
        f'Actual gear: {actual_gear} | mode: {mode} | A: {a_switch}',
        f'Actual speed: {actual_speed} m/s | steering: {actual_steer} rad',
        'w: apply speed | a/d: left/right steering only',
        f's: speed 0, brake {controller.stop_brake_percent:g}%, steering center (prepare control)',
        f'q/z: speed +/-{controller.speed_step:g} m/s | Ctrl+C: exit',
        'Gear keys: 1=P  2=D  3=N  4=R (stop before changing)',
        command,
        actuator,
        f'Phase: {controller.phase} | Pending gear: {_gear(pending_gear)}',
        driver_text,
        f'Last key: {last_key} | Input: {timeout}',
        limits,
        'Input timeout and exit stop publishing; no automatic brake.',
        f'Notice: {controller.notice}',
    ]


def _clip(text, width):
    """Strip terminal controls and fit text inside a terminal cell count."""
    result = []
    cells = 0
    for char in str(text):
        category = unicodedata.category(char)
        if category.startswith('C') or char in ('\u2028', '\u2029'):
            char, category = ' ', 'Zs'
        size = 0 if unicodedata.combining(char) else (
            2 if unicodedata.east_asian_width(char) in ('W', 'F') else 1)
        if cells + size > width:
            break
        if size == 0 and not result:
            continue
        result.append(char)
        cells += size
    return ''.join(result)


class TerminalScreen:
    """Draw to an owned terminal descriptor without blocking the ROS loop."""

    def __init__(self, fd):
        self.fd = os.dup(fd)
        self.was_blocking = os.get_blocking(self.fd)
        self.pending = ENTER_SCREEN
        self.size = None
        self.previous_rows = 0
        self.disconnected = False
        try:
            os.set_blocking(self.fd, False)
            self._flush()
        except BaseException:
            self.close()
            raise

    def _flush(self):
        if not self.pending or self.disconnected:
            return not self.pending
        try:
            count = os.write(self.fd, self.pending)
        except BlockingIOError:
            return False
        except OSError as error:
            if error.errno not in (errno.EIO, errno.ENXIO):
                raise
            self.disconnected = True
            return False
        self.pending = self.pending[count:]
        return not self.pending

    def render(self, controller, now, ros_ns, last_key='-'):
        """Render one bounded frame, retaining partial writes until next call."""
        if self.fd is None or self.disconnected or not self._flush():
            return False
        try:
            columns, rows = os.get_terminal_size(self.fd)
        except OSError:
            columns, rows = 80, 24
        if columns <= 0 or rows <= 0:
            columns, rows = 80, 24
        # Leave the last row and column unused to prevent terminal autoscroll.
        width, height = max(0, columns - 1), max(0, rows - 1)
        lines = format_lines(controller, now, ros_ns, last_key)[:height]
        resized = (columns, rows) != self.size
        frame = '\x1b[2J' if resized else ''
        for row in range(min(height, max(len(lines), self.previous_rows))):
            line = lines[row] if row < len(lines) else ''
            # Limit extremely wide terminals too, keeping each frame bounded.
            frame += f'\x1b[{row + 1};1H\x1b[2K' + _clip(line, min(width, 100))
        self.size = (columns, rows)
        self.previous_rows = len(lines)
        self.pending = frame.encode('utf-8')
        return self._flush()

    def close(self):
        """Restore cursor and screen; tolerate terminal hangup and backpressure."""
        if self.fd is None:
            return
        try:
            try:
                # CAN cancels any incomplete escape sequence from a short write.
                os.write(self.fd, LEAVE_SCREEN)
            except BlockingIOError:
                pass
            except OSError as error:
                if error.errno not in (errno.EIO, errno.ENXIO):
                    raise
        finally:
            try:
                os.set_blocking(self.fd, self.was_blocking)
            finally:
                os.close(self.fd)
                self.fd = None

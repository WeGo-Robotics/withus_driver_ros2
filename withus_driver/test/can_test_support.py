"""Shared socket and ROS helpers for isolated virtual CAN integration tests.

Only an existing kernel-kind vcan interface is accepted. These helpers never
configure interfaces, launch a hardware connection, or stop unrelated nodes.
"""

from collections import deque
from dataclasses import dataclass
import json
import os
import select
import signal
import socket
import struct
import subprocess
import threading
import time

import rclpy
from withus_msgs.msg import BodyCommand, DriveCommand, DriverStatus, VehicleStatus
from withus_msgs.srv import SetControl

CAN_FRAME = struct.Struct("=IB3x8s")
COMMAND_IDS = (0x501, 0x502, 0x503, 0x504, 0x506)
FEEDBACK_IDS = (0x10A, 0x301, 0x303, 0x304)


def require_vcan(name):
    result = subprocess.run(
        ["ip", "-j", "-details", "link", "show", "dev", name],
        text=True, capture_output=True, timeout=2.0, check=False)
    links = json.loads(result.stdout) if result.returncode == 0 else []
    if len(links) != 1 or links[0].get("linkinfo", {}).get("info_kind") != "vcan":
        raise ValueError("Test requires kernel interface kind vcan: " + name)
    if "UP" not in links[0].get("flags", []):
        raise ValueError("Test requires an already-UP vcan interface: " + name)
    return name


@dataclass(frozen=True)
class Observation:
    at: float
    can_id: int
    length: int
    data: bytes


class CanPeer:
    """Socket/thread lifecycle; subclasses supply synthetic receive and feedback."""

    def __init__(self, interface):
        self.interface = require_vcan(interface)
        self.lock = threading.Lock()
        self.quit = threading.Event()
        self.thread = None
        self.sock = None
        self.error = None
        self.observations = deque(maxlen=50000)
        self.counters = {can_id: 0 for can_id in FEEDBACK_IDS}
        self.values = {
            "a_raw": 1, "paused": set(), "deny_ad": False,
            "refuse_remote": False, "mode_override": None,
            "gear": 2, "speed": 0.0, "fault": False,
        }
        self.mode = 0

    def start(self):
        self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        try:
            mask = 0xC00007FF
            filters = b"".join(struct.pack("=II", can_id, mask) for can_id in COMMAND_IDS)
            self.sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, filters)
            self.sock.bind((self.interface,))
            self.sock.setblocking(False)
        except BaseException:
            self.sock.close()
            raise
        self.thread = threading.Thread(target=self._run, name="virtual_can_peer", daemon=True)
        self.thread.start()
        return self

    def close(self):
        self.quit.set()
        if self.thread:
            self.thread.join(timeout=2.0)
            if self.thread.is_alive():
                raise RuntimeError("Virtual peer did not stop")
        if self.sock:
            self.sock.close()
            self.sock = None

    def inject(self, **values):
        if not set(values).issubset(self.values):
            raise ValueError("Unknown peer injection")
        if "paused" in values:
            values["paused"] = set(values["paused"])
            if not values["paused"].issubset(FEEDBACK_IDS):
                raise ValueError("Unknown paused feedback ID")
        with self.lock:
            self.values.update(values)

    def history(self, since=0.0):
        with self.lock:
            return [frame for frame in self.observations if frame.at >= since]

    def _run(self):
        next_feedback = time.monotonic()
        try:
            while not self.quit.is_set():
                now = time.monotonic()
                if now >= next_feedback:
                    self._feedback()
                    next_feedback += 0.02
                    if next_feedback < now:
                        next_feedback = now + 0.02
                ready, _, _ = select.select(
                    [self.sock], [], [], max(0.0, min(0.01, next_feedback - time.monotonic())))
                if ready:
                    self._receive()
        except BaseException as error:
            self.error = error
            self.quit.set()


class RosHarness:
    def __init__(self, executable, peer):
        self.executable = executable
        self.peer = peer
        self.namespace = "/withus_test_" + str(os.getpid())
        self.node = rclpy.create_node("test_client", namespace=self.namespace)
        self.status = None
        self.vehicle = None
        self.process = None
        self.log = None
        self.logs = []
        self.checks = 0
        self.drive = self.node.create_publisher(DriveCommand, "drive_command", 10)
        self.body = self.node.create_publisher(BodyCommand, "body_command", 10)
        self.status_subscription = self.node.create_subscription(
            DriverStatus, "driver_status", self._status, 10)
        self.vehicle_subscription = self.node.create_subscription(
            VehicleStatus, "vehicle_status", self._vehicle, 10)
        self.service = self.node.create_client(SetControl, "set_control")

    def _status(self, msg):
        self.status = msg

    def _vehicle(self, msg):
        self.vehicle = msg

    def check(self, condition, description):
        if not condition:
            reason = self.status.reason if self.status else "no DriverStatus"
            raise AssertionError(description + "; driver reason: " + reason)
        self.checks += 1
        print("PASS " + description, flush=True)

    def step(self):
        if self.peer.error:
            raise RuntimeError("Virtual peer failed: " + str(self.peer.error))
        if self.process and self.process.poll() is not None:
            raise RuntimeError("Driver exited unexpectedly; see " + self.logs[-1])
        rclpy.spin_once(self.node, timeout_sec=0.005)

    def wait(self, predicate, description, timeout=2.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.step()
            if predicate():
                self.check(True, description)
                return
        self.check(False, "Timed out: " + description)

    def pump(self, duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            self.step()

    def stop(self, expected_code=0):
        if self.process:
            process = self.process
            self.process = None
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
            try:
                code = process.wait(timeout=4.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2.0)
                raise AssertionError("Owned driver did not shut down within 4 seconds")
            finally:
                if self.log:
                    self.log.close()
                    self.log = None
            self.check(code == expected_code, "driver SIGINT exit code is " + str(expected_code))
        for _ in range(25):
            rclpy.spin_once(self.node, timeout_sec=0.005)

    def close(self):
        if self.process:
            process = self.process
            self.process = None
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=4.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2.0)
        if self.log:
            self.log.close()
        self.node.destroy_node()

    def control(self, enabled, accepted=True):
        request = SetControl.Request()
        request.enable = enabled
        future = self.service.call_async(request)
        self.wait(future.done, "SetControl response received")
        result = future.result()
        self.check(result.accepted == accepted,
                   "SetControl({}) accepted={} ({})".format(enabled, accepted, result.message))
        return result

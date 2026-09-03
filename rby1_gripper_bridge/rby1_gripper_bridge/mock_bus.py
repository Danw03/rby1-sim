"""In-process Dynamixel stand-in for ROS wiring and UI/backend tests."""

from __future__ import annotations

from dataclasses import dataclass
import math
import threading
from typing import Sequence


@dataclass
class MockMotorState:
    torque_enable: bool = False
    position: float = 0.0
    velocity: float = 0.0
    current: float = 0.0
    temperature: int = 25


class MockDynamixelBus:
    """A deterministic loopback bus; it does not model the MuJoCo gripper."""

    current_control_mode = 0
    current_based_position_control_mode = 5

    def __init__(self, motor_ids: Sequence[int] = (0, 1)) -> None:
        self._ids = tuple(int(value) for value in motor_ids)
        self._lock = threading.Lock()
        self._opened = False
        self._closed = False
        self._mode = {
            motor_id: self.current_based_position_control_mode
            for motor_id in self._ids
        }
        self._enabled = {motor_id: False for motor_id in self._ids}
        self._positions = {motor_id: 0.0 for motor_id in self._ids}
        self._velocities = {motor_id: 0.0 for motor_id in self._ids}
        self._currents = {motor_id: 0.0 for motor_id in self._ids}
        self._target_positions = {motor_id: 0.0 for motor_id in self._ids}
        self._target_torques = {motor_id: 0.0 for motor_id in self._ids}

    def open_port(self) -> bool:
        self._opened = True
        self._closed = False
        return True

    def close_port(self) -> None:
        self._closed = True

    def set_baud_rate(self, _baud_rate: int) -> bool:
        return self._opened and not self._closed

    def set_torque_constant(self, _values) -> None:
        return None

    def ping(self, motor_id: int) -> bool:
        return motor_id in self._ids and self._opened and not self._closed

    def group_sync_write_torque_enable(self, ids, enabled: int) -> None:
        with self._lock:
            for motor_id in ids:
                self._enabled[int(motor_id)] = bool(enabled)

    def group_sync_write_operating_mode(self, values) -> None:
        with self._lock:
            for motor_id, mode in values:
                self._mode[int(motor_id)] = int(mode)

    def group_sync_write_send_torque(self, values) -> None:
        with self._lock:
            for motor_id, torque in values:
                self._target_torques[int(motor_id)] = float(torque)

    def group_sync_write_send_position(self, values) -> None:
        with self._lock:
            for motor_id, position in values:
                self._target_positions[int(motor_id)] = float(position)

    def group_fast_sync_read_encoder(self, ids):
        with self._lock:
            self._step()
            return [
                (int(motor_id), self._positions[int(motor_id)])
                for motor_id in ids
            ]

    def get_motor_states(self, ids):
        with self._lock:
            self._step()
            return [
                (
                    int(motor_id),
                    MockMotorState(
                        torque_enable=self._enabled[int(motor_id)],
                        position=self._positions[int(motor_id)],
                        velocity=self._velocities[int(motor_id)],
                        current=self._currents[int(motor_id)],
                        temperature=25,
                    ),
                )
                for motor_id in ids
            ]

    def _step(self) -> None:
        for motor_id in self._ids:
            before = self._positions[motor_id]
            if not self._enabled[motor_id]:
                after = before
                current = 0.0
            elif self._mode[motor_id] == self.current_control_mode:
                torque = self._target_torques[motor_id]
                after = max(-1.0, min(1.0, before + torque * 0.5))
                current = torque
            else:
                delta = self._target_positions[motor_id] - before
                step = max(-0.1, min(0.1, delta))
                after = before + step
                current = min(1.0, abs(delta))
            self._positions[motor_id] = after
            self._velocities[motor_id] = after - before
            self._currents[motor_id] = current
            if not math.isfinite(after):
                raise RuntimeError('mock gripper state became invalid')

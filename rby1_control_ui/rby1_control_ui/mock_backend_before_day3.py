"""Mock backend for RB-Y1 Control UI."""
from __future__ import annotations

from collections import deque
import threading
import time
from typing import Deque, List, Tuple

from .ros_backend import BackendSnapshot, VelocityCommand


class MockRby1Backend:
    backend_name = 'MOCK'

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._events: Deque[Tuple[str, str]] = deque(maxlen=300)
        self._command = VelocityCommand()
        self._last_command_update = time.monotonic()
        self.control_state = 0
        self.power_enabled = False
        self.servo_enabled = False
        self.stream_enabled = False
        self.emo_active = False
        self.collision_active = False
        self.services_enabled = True
        self._push_event('info', 'Mock backend started.')

    def _push_event(self, level: str, message: str) -> None:
        stamp = time.strftime('%H:%M:%S')
        with self._lock:
            self._events.append((level, f'[{stamp}] {message}'))

    def drain_events(self) -> List[Tuple[str, str]]:
        with self._lock:
            events = list(self._events)
            self._events.clear()
        return events

    def set_velocity(self, vx: float, vy: float, wz: float) -> None:
        cmd = VelocityCommand(float(vx), float(vy), float(wz))
        with self._lock:
            changed = cmd != self._command
            self._command = cmd
            self._last_command_update = time.monotonic()
        if changed:
            self._push_event(
                'info',
                f'MOCK cmd_vel: vx={cmd.vx:+.3f}, vy={cmd.vy:+.3f}, wz={cmd.wz:+.3f}',
            )

    def stop(self, publish_immediately: bool = True) -> None:
        del publish_immediately
        self.set_velocity(0.0, 0.0, 0.0)

    def prepare_robot(self) -> None:
        self.request_power(True)
        self.request_servo(True)
        self._push_event('info', 'Mock robot preparation completed.')

    def request_power(self, enabled: bool, target: str = 'all') -> None:
        del target
        self.power_enabled = bool(enabled)
        if not self.power_enabled:
            self.servo_enabled = False
            self.stream_enabled = False
            self.control_state = 0
            self.stop()
        else:
            self.control_state = 1
        self._push_event('info', f'MOCK Power {"ON" if enabled else "OFF"}.')

    def request_servo(self, enabled: bool, target: str = 'all') -> None:
        del target
        enabled = bool(enabled)
        if enabled and not self.power_enabled:
            self._push_event('error', 'MOCK Servo ON rejected: Power is OFF.')
            return
        self.servo_enabled = enabled
        self.control_state = 2 if enabled else (1 if self.power_enabled else 0)
        if not enabled:
            self.stream_enabled = False
            self.stop()
        self._push_event('info', f'MOCK Servo {"ON" if enabled else "OFF"}.')

    def request_stream(self, enabled: bool, value: float = 0.0) -> None:
        del value
        enabled = bool(enabled)
        if enabled and not self.servo_enabled:
            self._push_event('error', 'MOCK Stream ON rejected: Servo is OFF.')
            return
        self.stream_enabled = enabled
        if not enabled:
            self.stop()
        self._push_event('info', f'MOCK Stream {"ON" if enabled else "OFF"}.')

    def snapshot(self) -> BackendSnapshot:
        with self._lock:
            cmd = self._command
            age = time.monotonic() - self._last_command_update
        return BackendSnapshot(
            namespace='/mock',
            cmd_vel_topic='/mock/cmd_vel',
            cmd_vel_subscribers=1,
            control_state=self.control_state,
            stream_enabled=self.stream_enabled,
            emo_active=self.emo_active,
            collision_active=self.collision_active,
            services_enabled=True,
            rby1_msgs_available=True,
            service_ready={'power': True, 'servo': True, 'stream': True},
            command=cmd,
            command_stale=age > 0.35,
        )

    def run_diagnostics(self) -> dict:
        s = self.snapshot()
        return {
            'backend': self.backend_name,
            'namespace': s.namespace,
            'cmd_vel_topic': s.cmd_vel_topic,
            'cmd_vel_subscribers': s.cmd_vel_subscribers,
            'control_state': s.control_state,
            'stream_enabled': s.stream_enabled,
            'emo_active': s.emo_active,
            'collision_active': s.collision_active,
            'services_enabled': s.services_enabled,
            'service_ready': dict(s.service_ready),
            'command': {'vx': s.command.vx, 'vy': s.command.vy, 'wz': s.command.wz},
        }

    def shutdown_safely(self, turn_stream_off: bool = True) -> None:
        self.stop()
        if turn_stream_off:
            self.stream_enabled = False
        self._push_event('info', 'Mock backend shut down safely.')

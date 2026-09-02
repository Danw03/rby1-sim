"""Mock backend for RB-Y1 Control UI.

Day 2 additions:
- joint group state
- Cartesian end-effector state
- joint step jog
- joint target move
- Cartesian step jog
- Cartesian target move
- global arm-motion cancel

The values are simulation-only UI test data. They are not a dynamics model.
"""
from __future__ import annotations

from collections import deque
import threading
import time
from typing import Deque, List, Tuple

from .backend_contract import (
    BackendSnapshot,
    TaskBackendState,
    TaskCommandState,
    TaskCommandStatus,
    VelocityCommand,
)
from .task_commands import CommandKind, TaskCommand


class MockRby1Backend:
    backend_name = "MOCK"

    JOINT_GROUP_DOF = {
        "right_arm": 7,
        "left_arm": 7,
        "torso": 6,
        "head": 2,
    }

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._events: Deque[Tuple[str, str]] = deque(maxlen=300)

        # Base state
        self._command = VelocityCommand()
        self._last_command_update = time.monotonic()

        self.control_state = 0
        self.power_enabled = False
        self.servo_enabled = False
        self.stream_enabled = False
        self.emo_active = False
        self.collision_active = False
        self.services_enabled = True

        # Day 2 mock motion state.
        # Joint UI uses degrees for readability in mock mode.
        self._joint_groups = {
            key: [0.0] * dof
            for key, dof in self.JOINT_GROUP_DOF.items()
        }

        # Cartesian UI convention for Day 2:
        # [x, y, z, roll, pitch, yaw]
        # x/y/z [m], roll/pitch/yaw [deg].
        # These are UI test values only, not measured RB-Y1 poses.
        self._cartesian = {
            "right_arm": [0.0] * 6,
            "left_arm": [0.0] * 6,
        }

        self._motion_active = False
        self._motion_description = ""
        self._task_command_sequence = 0
        self._task_commands = {}

        self._push_event("info", "Mock backend started.")

    # ==================================================================
    # Events
    # ==================================================================
    def _push_event(self, level: str, message: str) -> None:
        stamp = time.strftime("%H:%M:%S")
        with self._lock:
            self._events.append(
                (level, f"[{stamp}] {message}")
            )

    def drain_events(self) -> List[Tuple[str, str]]:
        with self._lock:
            events = list(self._events)
            self._events.clear()
        return events

    # ==================================================================
    # Base
    # ==================================================================
    def set_velocity(
        self,
        vx: float,
        vy: float,
        wz: float,
    ) -> None:
        cmd = VelocityCommand(
            float(vx),
            float(vy),
            float(wz),
        )

        with self._lock:
            changed = cmd != self._command
            self._command = cmd
            self._last_command_update = time.monotonic()

        if changed:
            self._push_event(
                "info",
                (
                    "MOCK cmd_vel: "
                    f"vx={cmd.vx:+.3f}, "
                    f"vy={cmd.vy:+.3f}, "
                    f"wz={cmd.wz:+.3f}"
                ),
            )

    def stop(
        self,
        publish_immediately: bool = True,
    ) -> None:
        del publish_immediately
        self.set_velocity(0.0, 0.0, 0.0)

    # ==================================================================
    # Robot preparation / services
    # ==================================================================
    def prepare_robot(self) -> None:
        self.request_power(True)
        self.request_servo(True)
        self._push_event(
            "info",
            "Mock robot preparation completed.",
        )

    def request_power(
        self,
        enabled: bool,
        target: str = "all",
    ) -> None:
        del target

        self.power_enabled = bool(enabled)

        if not self.power_enabled:
            self.servo_enabled = False
            self.stream_enabled = False
            self.control_state = 0
            self.stop()
            self.cancel_motion()
        else:
            self.control_state = 1

        self._push_event(
            "info",
            f'MOCK Power {"ON" if enabled else "OFF"}.',
        )

    def request_servo(
        self,
        enabled: bool,
        target: str = "all",
    ) -> None:
        del target
        enabled = bool(enabled)

        if enabled and not self.power_enabled:
            self._push_event(
                "error",
                "MOCK Servo ON rejected: Power is OFF.",
            )
            return

        self.servo_enabled = enabled
        self.control_state = (
            2
            if enabled
            else (1 if self.power_enabled else 0)
        )

        if not enabled:
            self.stream_enabled = False
            self.stop()
            self.cancel_motion()

        self._push_event(
            "info",
            f'MOCK Servo {"ON" if enabled else "OFF"}.',
        )

    def request_stream(
        self,
        enabled: bool,
        value: float = 0.0,
    ) -> None:
        del value
        enabled = bool(enabled)

        if enabled and not self.servo_enabled:
            self._push_event(
                "error",
                "MOCK Stream ON rejected: Servo is OFF.",
            )
            return

        self.stream_enabled = enabled

        if not enabled:
            self.stop()

        self._push_event(
            "info",
            f'MOCK Stream {"ON" if enabled else "OFF"}.',
        )

    # ==================================================================
    # Day 2 motion backend contract
    # ==================================================================
    def _motion_ready(self) -> bool:
        if not self.power_enabled:
            self._push_event(
                "error",
                "MOCK arm motion rejected: Power is OFF.",
            )
            return False

        if not self.servo_enabled:
            self._push_event(
                "error",
                "MOCK arm motion rejected: Servo is OFF.",
            )
            return False

        if self.emo_active:
            self._push_event(
                "error",
                "MOCK arm motion rejected: EMO is active.",
            )
            return False

        if self.collision_active:
            self._push_event(
                "error",
                "MOCK arm motion rejected: Collision state is active.",
            )
            return False

        return True

    def get_motion_state(self) -> dict:
        with self._lock:
            return {
                "joint_groups": {
                    key: list(values)
                    for key, values in self._joint_groups.items()
                },
                "cartesian": {
                    key: list(values)
                    for key, values in self._cartesian.items()
                },
                "motion_active": bool(self._motion_active),
                "motion_description": self._motion_description,
            }

    def jog_joint(
        self,
        group: str,
        joint_index: int,
        delta_deg: float,
    ) -> None:
        if not self._motion_ready():
            return

        if group not in self._joint_groups:
            self._push_event(
                "error",
                f"MOCK joint jog rejected: unknown group '{group}'.",
            )
            return

        values = self._joint_groups[group]

        if not 0 <= int(joint_index) < len(values):
            self._push_event(
                "error",
                (
                    "MOCK joint jog rejected: "
                    f"joint index {joint_index} is invalid for {group}."
                ),
            )
            return

        with self._lock:
            values[int(joint_index)] += float(delta_deg)
            value = values[int(joint_index)]
            self._motion_active = False
            self._motion_description = ""

        self._push_event(
            "info",
            (
                f"MOCK Joint jog: {group}[{joint_index}] "
                f"{float(delta_deg):+.2f} deg "
                f"→ {value:+.2f} deg"
            ),
        )

    def move_joint_group(
        self,
        group: str,
        target_deg,
        minimum_time: float = 3.0,
    ) -> None:
        if not self._motion_ready():
            return

        if group not in self._joint_groups:
            self._push_event(
                "error",
                f"MOCK joint target rejected: unknown group '{group}'.",
            )
            return

        expected = len(self._joint_groups[group])
        values = [float(v) for v in target_deg]

        if len(values) != expected:
            self._push_event(
                "error",
                (
                    "MOCK joint target rejected: "
                    f"{group} expects {expected} values, "
                    f"got {len(values)}."
                ),
            )
            return

        with self._lock:
            self._motion_active = True
            self._motion_description = (
                f"Joint target: {group}"
            )

            # Day 2 mock completes immediately.
            self._joint_groups[group] = values
            self._motion_active = False
            self._motion_description = ""

        self._push_event(
            "info",
            (
                f"MOCK Joint target: {group}, "
                f"minimum_time={float(minimum_time):.1f}s, "
                f"target={values}"
            ),
        )

    def jog_cartesian(
        self,
        arm: str,
        axis_index: int,
        delta: float,
        reference_frame: str = "base",
    ) -> None:
        if not self._motion_ready():
            return

        if arm not in self._cartesian:
            self._push_event(
                "error",
                f"MOCK Cartesian jog rejected: unknown arm '{arm}'.",
            )
            return

        if not 0 <= int(axis_index) < 6:
            self._push_event(
                "error",
                (
                    "MOCK Cartesian jog rejected: "
                    f"axis index {axis_index} is invalid."
                ),
            )
            return

        with self._lock:
            self._cartesian[arm][int(axis_index)] += float(delta)
            value = self._cartesian[arm][int(axis_index)]

        unit = "m" if int(axis_index) < 3 else "deg"

        self._push_event(
            "info",
            (
                f"MOCK Cartesian jog: {arm} axis={axis_index}, "
                f"delta={float(delta):+.4f} {unit}, "
                f"value={value:+.4f} {unit}, "
                f"frame={reference_frame}"
            ),
        )

    def move_cartesian(
        self,
        arm: str,
        target_pose,
        minimum_time: float = 3.0,
        reference_frame: str = "base",
    ) -> None:
        if not self._motion_ready():
            return

        if arm not in self._cartesian:
            self._push_event(
                "error",
                f"MOCK Cartesian target rejected: unknown arm '{arm}'.",
            )
            return

        values = [float(v) for v in target_pose]

        if len(values) != 6:
            self._push_event(
                "error",
                (
                    "MOCK Cartesian target rejected: "
                    f"expected 6 values, got {len(values)}."
                ),
            )
            return

        with self._lock:
            self._motion_active = True
            self._motion_description = (
                f"Cartesian target: {arm}"
            )

            # Day 2 mock completes immediately.
            self._cartesian[arm] = values
            self._motion_active = False
            self._motion_description = ""

        self._push_event(
            "info",
            (
                f"MOCK Cartesian target: {arm}, "
                f"minimum_time={float(minimum_time):.1f}s, "
                f"frame={reference_frame}, target={values}"
            ),
        )

    def cancel_motion(self) -> None:
        with self._lock:
            was_active = self._motion_active
            self._motion_active = False
            self._motion_description = ""

        if was_active:
            self._push_event(
                "warning",
                "MOCK arm motion cancelled.",
            )
        else:
            self._push_event(
                "info",
                "MOCK arm motion cancel requested.",
            )

    # ==================================================================
    # Asynchronous Task contract (immediate completion in Mock mode)
    # ==================================================================
    def task_state(self) -> TaskBackendState:
        now = time.monotonic()
        with self._lock:
            return TaskBackendState(
                captured_at=now,
                robot_state_updated_at=now,
                control_state=self.control_state,
                emo_active=self.emo_active,
                collision_active=self.collision_active,
                motion_active=bool(self._motion_active),
                joint_groups={
                    key: tuple(values)
                    for key, values in self._joint_groups.items()
                },
                joint_updated_at={
                    key: now for key in self._joint_groups
                },
                joint_order_verified={
                    key: True for key in self._joint_groups
                },
                cartesian={
                    key: tuple(values)
                    for key, values in self._cartesian.items()
                },
                cartesian_updated_at={
                    key: now for key in self._cartesian
                },
                driver_safety_verified=True,
                driver_safety_updated_at=now,
            )

    def start_task_command(self, command: TaskCommand) -> str:
        if not self._motion_ready():
            raise RuntimeError("Mock robot is not ready for Task motion")
        if not isinstance(command, TaskCommand):
            raise TypeError("command must be a TaskCommand")
        if command.kind not in (
            CommandKind.JOINT_ABSOLUTE,
            CommandKind.JOINT_ABSOLUTE_MULTI,
            CommandKind.LINEAR_ABSOLUTE,
        ):
            raise ValueError("backend accepts only resolved absolute commands")

        with self._lock:
            self._task_command_sequence += 1
            command_id = f"mock-{self._task_command_sequence}"
            if command.kind in (
                CommandKind.JOINT_ABSOLUTE,
                CommandKind.JOINT_ABSOLUTE_MULTI,
            ):
                joint_targets = (
                    command.joint_targets
                    if command.kind is CommandKind.JOINT_ABSOLUTE_MULTI
                    else ((str(command.group), command.values),)
                )
                for group, values in joint_targets:
                    self._joint_groups[group] = list(values)
            else:
                self._cartesian[str(command.group)] = list(command.values)

            self._task_commands[command_id] = TaskCommandState(
                TaskCommandStatus.SUCCEEDED,
                "Mock command completed",
            )

        self._push_event(
            "info",
            f"MOCK Task command completed: {command.kind.value}",
        )
        return command_id

    def poll_task_command(self, command_id: str) -> TaskCommandState:
        with self._lock:
            try:
                return self._task_commands[command_id]
            except KeyError as exc:
                raise KeyError(f"unknown Task command: {command_id}") from exc

    def cancel_task_command(self, command_id: str) -> None:
        with self._lock:
            if command_id in self._task_commands:
                self._task_commands[command_id] = TaskCommandState(
                    TaskCommandStatus.CANCELED,
                    "Canceled by operator",
                )
        self.cancel_motion()

    # ==================================================================
    # Snapshot / diagnostics / shutdown
    # ==================================================================
    def snapshot(self) -> BackendSnapshot:
        with self._lock:
            cmd = self._command
            age = (
                time.monotonic()
                - self._last_command_update
            )

        return BackendSnapshot(
            namespace="/mock",
            cmd_vel_topic="/mock/cmd_vel",
            cmd_vel_subscribers=1,
            control_state=self.control_state,
            power_enabled=self.power_enabled,
            servo_enabled=self.servo_enabled,
            stream_enabled=self.stream_enabled,
            emo_active=self.emo_active,
            collision_active=self.collision_active,
            services_enabled=True,
            rby1_msgs_available=True,
            service_ready={
                "power": True,
                "servo": True,
                "stream": True,
                "control_manager": True,
                "joint_action": True,
                "cartesian_action": True,
                "cartesian_pose": True,
                "cancel_control": True,
                "scenario_safety": True,
            },
            command=cmd,
            command_stale=age > 0.35,
        )

    def run_diagnostics(self) -> dict:
        snapshot = self.snapshot()
        motion = self.get_motion_state()

        return {
            "backend": self.backend_name,
            "namespace": snapshot.namespace,
            "cmd_vel_topic": snapshot.cmd_vel_topic,
            "cmd_vel_subscribers": snapshot.cmd_vel_subscribers,
            "control_state": snapshot.control_state,
            "stream_enabled": snapshot.stream_enabled,
            "emo_active": snapshot.emo_active,
            "collision_active": snapshot.collision_active,
            "services_enabled": snapshot.services_enabled,
            "service_ready": dict(snapshot.service_ready),
            "command": {
                "vx": snapshot.command.vx,
                "vy": snapshot.command.vy,
                "wz": snapshot.command.wz,
            },
            "joint_groups": motion["joint_groups"],
            "cartesian": motion["cartesian"],
            "motion_active": motion["motion_active"],
        }

    def shutdown_safely(
        self,
        turn_stream_off: bool = True,
    ) -> None:
        self.stop()
        self.cancel_motion()

        if turn_stream_off:
            self.stream_enabled = False

        self._push_event(
            "info",
            "Mock backend shut down safely.",
        )

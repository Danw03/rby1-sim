"""ROS 2 backend for the RB-Y1 control UI.

The backend keeps the Qt GUI independent from ROS message handling.
It publishes geometry_msgs/Twist continuously and optionally uses RB-Y1
power, servo, stream-control services, and robot-state feedback.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
import threading
import time
from typing import Deque, Dict, List, Optional, Tuple

from geometry_msgs.msg import Twist
from rclpy.node import Node

try:
    from rby1_msgs.msg import RobotState
    from rby1_msgs.srv import StateOnOff

    RBY1_MSGS_AVAILABLE = True
except ImportError:
    RobotState = None  # type: ignore[assignment]
    StateOnOff = None  # type: ignore[assignment]
    RBY1_MSGS_AVAILABLE = False


@dataclass(frozen=True)
class VelocityCommand:
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0

    @property
    def stopped(self) -> bool:
        return (
            abs(self.vx) < 1e-9
            and abs(self.vy) < 1e-9
            and abs(self.wz) < 1e-9
        )


@dataclass(frozen=True)
class BackendSnapshot:
    namespace: str
    cmd_vel_topic: str
    cmd_vel_subscribers: int

    control_state: Optional[int]
    stream_enabled: Optional[bool]
    emo_active: Optional[bool]
    collision_active: Optional[bool]

    services_enabled: bool
    rby1_msgs_available: bool
    service_ready: Dict[str, bool]

    command: VelocityCommand
    command_stale: bool


class Rby1ControlNode(Node):
    """ROS node used by the Qt interface."""

    def __init__(self) -> None:
        super().__init__('rby1_control_ui')

        # ROS topic and service names.
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('robot_state_topic', 'robot_state')
        self.declare_parameter('robot_power_service', 'robot_power')
        self.declare_parameter('robot_servo_service', 'robot_servo')
        self.declare_parameter(
            'stream_control_service',
            'stream_control',
        )

        # Backend behavior.
        self.declare_parameter('use_rby1_services', True)
        self.declare_parameter('publish_rate_hz', 25.0)
        self.declare_parameter('command_timeout_sec', 0.35)
        self.declare_parameter('publish_zero_when_idle', True)

        self.cmd_vel_topic = str(
            self.get_parameter('cmd_vel_topic').value
        )
        self.robot_state_topic = str(
            self.get_parameter('robot_state_topic').value
        )
        self.robot_power_service = str(
            self.get_parameter('robot_power_service').value
        )
        self.robot_servo_service = str(
            self.get_parameter('robot_servo_service').value
        )
        self.stream_control_service = str(
            self.get_parameter('stream_control_service').value
        )

        requested_services = bool(
            self.get_parameter('use_rby1_services').value
        )

        self.services_enabled = (
            requested_services and RBY1_MSGS_AVAILABLE
        )

        self.publish_rate_hz = self._positive_float(
            self.get_parameter('publish_rate_hz').value,
            fallback=25.0,
        )

        self.command_timeout_sec = self._positive_float(
            self.get_parameter('command_timeout_sec').value,
            fallback=0.35,
        )

        self.publish_zero_when_idle = bool(
            self.get_parameter('publish_zero_when_idle').value
        )

        # Shared command state.
        self._lock = threading.RLock()

        self._command = VelocityCommand()
        self._last_command_update = time.monotonic()

        self._last_published = VelocityCommand()
        self._last_logged_command: Optional[VelocityCommand] = None

        self._events: Deque[Tuple[str, str]] = deque(maxlen=300)

        # Actual robot state received from /robot_state.
        self.control_state: Optional[int] = None
        self.stream_enabled: Optional[bool] = None
        self.emo_active: Optional[bool] = None
        self.collision_active: Optional[bool] = None

        self._robot_state_received = False

        # Prepare Robot state machine.
        self._prepare_stage = 'idle'
        self._prepare_not_before = 0.0
        self._prepare_deadline = 0.0

        # Keep asynchronous service futures alive.
        self._pending_futures: List[object] = []

        # cmd_vel publisher.
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            10,
        )

        # RB-Y1 service clients and subscriber.
        self.power_client = None
        self.servo_client = None
        self.stream_client = None
        self.state_sub = None

        if self.services_enabled:
            assert StateOnOff is not None
            assert RobotState is not None

            self.power_client = self.create_client(
                StateOnOff,
                self.robot_power_service,
            )

            self.servo_client = self.create_client(
                StateOnOff,
                self.robot_servo_service,
            )

            self.stream_client = self.create_client(
                StateOnOff,
                self.stream_control_service,
            )

            self.state_sub = self.create_subscription(
                RobotState,
                self.robot_state_topic,
                self._state_callback,
                10,
            )

        elif requested_services:
            self._push_event(
                'warning',
                'rby1_msgs를 불러오지 못해 '
                'cmd_vel 전용 모드로 시작합니다.',
            )

        # Continuous Twist publisher.
        self.publish_timer = self.create_timer(
            1.0 / self.publish_rate_hz,
            self._publish_cycle,
        )

        # Power -> Servo preparation state machine.
        self.operation_timer = self.create_timer(
            0.05,
            self._process_prepare_operation,
        )

        self._push_event(
            'info',
            f'ROS node started: '
            f'namespace={self.get_namespace()}, '
            f'cmd_vel={self.cmd_vel_pub.topic_name}',
        )

    @staticmethod
    def _positive_float(
        value: object,
        fallback: float,
    ) -> float:
        try:
            result = float(value)
        except (TypeError, ValueError):
            return fallback

        if not math.isfinite(result) or result <= 0.0:
            return fallback

        return result

    def _push_event(
        self,
        level: str,
        message: str,
    ) -> None:
        timestamp = time.strftime('%H:%M:%S')

        with self._lock:
            self._events.append(
                (level, f'[{timestamp}] {message}')
            )

        logger = self.get_logger()

        if level == 'error':
            logger.error(message)
        elif level == 'warning':
            logger.warning(message)
        else:
            logger.info(message)

    def drain_events(self) -> List[Tuple[str, str]]:
        with self._lock:
            items = list(self._events)
            self._events.clear()

        return items

    def _state_callback(self, msg) -> None:
        """Receive the actual state published by the RB-Y1 driver."""

        self._robot_state_received = True

        new_control_state = int(msg.control_manager_state)
        new_stream_enabled = bool(msg.robot_stream_state)
        new_emo_active = bool(msg.emo_state)
        new_collision_active = bool(msg.collision)

        if new_control_state != self.control_state:
            self.control_state = new_control_state

            self._push_event(
                'info',
                f'Robot control state changed to '
                f'{new_control_state}.',
            )

        if new_stream_enabled != self.stream_enabled:
            self.stream_enabled = new_stream_enabled

            self._push_event(
                'info',
                'Robot stream state changed to '
                f'{"ON" if new_stream_enabled else "OFF"}.',
            )

        if new_emo_active != self.emo_active:
            self.emo_active = new_emo_active

            self._push_event(
                'warning' if new_emo_active else 'info',
                'EMO state changed to '
                f'{"ACTIVE" if new_emo_active else "RELEASED"}.',
            )

            if new_emo_active:
                self.stop(publish_immediately=True)

        if new_collision_active != self.collision_active:
            self.collision_active = new_collision_active

            self._push_event(
                'warning' if new_collision_active else 'info',
                'Collision state changed to '
                f'{"ACTIVE" if new_collision_active else "CLEAR"}.',
            )

            if new_collision_active:
                self.stop(publish_immediately=True)

    def set_velocity(
        self,
        vx: float,
        vy: float,
        wz: float,
    ) -> None:
        """Update the target mobile-base velocity."""

        values = (
            float(vx),
            float(vy),
            float(wz),
        )

        if not all(
            math.isfinite(value)
            for value in values
        ):
            self._push_event(
                'error',
                'Invalid non-finite velocity command rejected.',
            )
            return

        with self._lock:
            self._command = VelocityCommand(*values)
            self._last_command_update = time.monotonic()

    def stop(
        self,
        publish_immediately: bool = True,
    ) -> None:
        """Set the velocity command to zero."""

        with self._lock:
            self._command = VelocityCommand()
            self._last_command_update = time.monotonic()

        if publish_immediately:
            self._publish_twist(VelocityCommand())

    def _current_command(
        self,
    ) -> Tuple[VelocityCommand, bool]:
        with self._lock:
            command = self._command
            age = (
                time.monotonic()
                - self._last_command_update
            )

        stale = age > self.command_timeout_sec

        return command, stale

    def _publish_cycle(self) -> None:
        """Publish Twist at the configured fixed rate."""

        command, stale = self._current_command()

        if stale:
            command = VelocityCommand()

        if (
            self.publish_zero_when_idle
            or not command.stopped
            or not self._last_published.stopped
        ):
            self._publish_twist(command)

    def _publish_twist(
        self,
        command: VelocityCommand,
    ) -> None:
        """Convert VelocityCommand to geometry_msgs/Twist."""

        msg = Twist()

        msg.linear.x = command.vx
        msg.linear.y = command.vy
        msg.angular.z = command.wz

        self.cmd_vel_pub.publish(msg)

        self._last_published = command

        # Log only when the command changes.
        # This makes lateral-command debugging easier.
        if command != self._last_logged_command:
            self._last_logged_command = command

            self._push_event(
                'info',
                'cmd_vel: '
                f'vx={command.vx:+.3f}, '
                f'vy={command.vy:+.3f}, '
                f'wz={command.wz:+.3f}',
            )

    def prepare_robot(self) -> None:
        """Run Power ON -> Servo ON as a non-blocking sequence."""

        if not self.services_enabled:
            self._push_event(
                'warning',
                'RB-Y1 service control is disabled.',
            )
            return

        if self._prepare_stage != 'idle':
            self._push_event(
                'warning',
                'Robot preparation is already running.',
            )
            return

        if self.control_state in (2, 3):
            self._push_event(
                'info',
                'Robot is already powered and servo-enabled.',
            )
            return

        self._prepare_stage = 'power_request'
        self._prepare_deadline = (
            time.monotonic() + 20.0
        )

        self._push_event(
            'info',
            'Starting robot preparation: '
            'Power ON -> Servo ON.',
        )

    def request_power(
        self,
        enabled: bool,
        target: str = 'all',
    ) -> None:
        """Request robot power ON or OFF."""

        if not enabled:
            self.stop(publish_immediately=True)

        self._request_state_on_off(
            client=self.power_client,
            enabled=enabled,
            parameters=target,
            value=0.0,
            label='Power',
        )

    def request_servo(
        self,
        enabled: bool,
        target: str = 'all',
    ) -> None:
        """Request robot servo ON or OFF."""

        if not enabled:
            self.stop(publish_immediately=True)

        self._request_state_on_off(
            client=self.servo_client,
            enabled=enabled,
            parameters=target,
            value=0.0,
            label='Servo',
        )

    def request_stream(
        self,
        enabled: bool,
        value: float = 0.0,
    ) -> None:
        """Request cmd_vel stream control ON or OFF."""

        if not enabled:
            self.stop(publish_immediately=True)

        self._request_state_on_off(
            client=self.stream_client,
            enabled=enabled,
            parameters='',
            value=value,
            label='Stream',
        )

    def _request_state_on_off(
        self,
        client,
        enabled: bool,
        parameters: str,
        value: float,
        label: str,
    ) -> None:
        """Common StateOnOff service request function."""

        if (
            not self.services_enabled
            or client is None
        ):
            self._push_event(
                'warning',
                f'{label} service is unavailable.',
            )
            return

        if not client.service_is_ready():
            self._push_event(
                'warning',
                f'{label} service not ready: '
                f'{client.srv_name}',
            )
            return

        assert StateOnOff is not None

        request = StateOnOff.Request()

        request.state = bool(enabled)
        request.parameters = str(parameters)
        request.value = float(value)

        future = client.call_async(request)

        self._pending_futures.append(future)

        future.add_done_callback(
            lambda done,
            service_label=label,
            requested=bool(enabled):
            self._state_on_off_done(
                done,
                service_label,
                requested,
            )
        )

        self._push_event(
            'info',
            f'{label} '
            f'{"ON" if enabled else "OFF"} '
            f'requested.',
        )

    def _state_on_off_done(
        self,
        future,
        label: str,
        enabled: bool,
    ) -> None:
        """Process Power, Servo, or Stream service response."""

        self._discard_future(future)

        try:
            result = future.result()
        except Exception as exc:
            self._push_event(
                'error',
                f'{label} service call failed: {exc}',
            )
            return

        if (
            result is not None
            and bool(result.success)
        ):
            # Normally stream state is taken from /robot_state.
            # Use the service result only as a fallback when the
            # robot-state topic has never been received.
            if (
                label == 'Stream'
                and not self._robot_state_received
            ):
                self.stream_enabled = enabled

            self._push_event(
                'info',
                f'{label} '
                f'{"ON" if enabled else "OFF"} '
                f'succeeded.',
            )
            return

        message = (
            getattr(result, 'message', 'No response')
            if result is not None
            else 'No response'
        )

        self._push_event(
            'error',
            f'{label} request failed: {message}',
        )

    def _process_prepare_operation(self) -> None:
        """Process the Power -> Servo preparation state machine."""

        if self._prepare_stage == 'idle':
            return

        now = time.monotonic()

        if now > self._prepare_deadline:
            self._push_event(
                'error',
                'Robot preparation timed out.',
            )

            self._prepare_stage = 'idle'
            return

        if self._prepare_stage == 'power_request':
            if (
                self.power_client is None
                or not self.power_client.service_is_ready()
            ):
                return

            self._call_state_service(
                self.power_client,
                state=True,
                parameters='all',
                callback=self._power_done,
            )

            self._prepare_stage = 'power_pending'

            self._push_event(
                'info',
                'Power ON request sent.',
            )

            return

        if self._prepare_stage == 'power_wait':
            if now >= self._prepare_not_before:
                self._prepare_stage = 'servo_request'

            return

        if self._prepare_stage == 'servo_request':
            if (
                self.servo_client is None
                or not self.servo_client.service_is_ready()
            ):
                return

            self._call_state_service(
                self.servo_client,
                state=True,
                parameters='all',
                callback=self._servo_done,
            )

            self._prepare_stage = 'servo_pending'

            self._push_event(
                'info',
                'Servo ON request sent.',
            )

            return

        if self._prepare_stage == 'wait_state':
            if self.control_state in (2, 3):
                self._push_event(
                    'info',
                    'Robot preparation completed.',
                )

                self._prepare_stage = 'idle'

            elif (
                self.control_state is None
                and now >= self._prepare_not_before
            ):
                self._push_event(
                    'warning',
                    'Servo request succeeded, but '
                    'robot_state is unavailable; '
                    'continuing without state confirmation.',
                )

                self._prepare_stage = 'idle'

    def _call_state_service(
        self,
        client,
        state: bool,
        parameters: str,
        callback,
    ) -> None:
        """Internal StateOnOff request used by Prepare Robot."""

        assert StateOnOff is not None

        request = StateOnOff.Request()

        request.state = bool(state)
        request.parameters = str(parameters)
        request.value = 0.0

        future = client.call_async(request)

        self._pending_futures.append(future)

        future.add_done_callback(callback)

    def _power_done(self, future) -> None:
        self._discard_future(future)

        result = self._safe_service_result(
            future,
            'Power ON',
        )

        if result:
            self._prepare_not_before = (
                time.monotonic() + 1.0
            )

            self._prepare_stage = 'power_wait'
        else:
            self._prepare_stage = 'idle'

    def _servo_done(self, future) -> None:
        self._discard_future(future)

        result = self._safe_service_result(
            future,
            'Servo ON',
        )

        if result:
            self._prepare_not_before = (
                time.monotonic() + 1.0
            )

            self._prepare_stage = 'wait_state'
        else:
            self._prepare_stage = 'idle'

    def _safe_service_result(
        self,
        future,
        name: str,
    ) -> bool:
        try:
            result = future.result()
        except Exception as exc:
            self._push_event(
                'error',
                f'{name} service failed: {exc}',
            )

            return False

        if (
            result is not None
            and bool(result.success)
        ):
            self._push_event(
                'info',
                f'{name} succeeded.',
            )

            return True

        message = (
            getattr(result, 'message', 'No response')
            if result is not None
            else 'No response'
        )

        self._push_event(
            'error',
            f'{name} failed: {message}',
        )

        return False

    def _discard_future(self, future) -> None:
        try:
            self._pending_futures.remove(future)
        except ValueError:
            pass

    def snapshot(self) -> BackendSnapshot:
        """Return a thread-safe state snapshot for the Qt UI."""

        command, stale = self._current_command()

        readiness = {
            'power': bool(
                self.power_client
                and self.power_client.service_is_ready()
            ),
            'servo': bool(
                self.servo_client
                and self.servo_client.service_is_ready()
            ),
            'stream': bool(
                self.stream_client
                and self.stream_client.service_is_ready()
            ),
        }

        return BackendSnapshot(
            namespace=self.get_namespace(),

            cmd_vel_topic=self.cmd_vel_pub.topic_name,

            cmd_vel_subscribers=self.count_subscribers(
                self.cmd_vel_pub.topic_name
            ),

            control_state=self.control_state,
            stream_enabled=self.stream_enabled,
            emo_active=self.emo_active,
            collision_active=self.collision_active,

            services_enabled=self.services_enabled,
            rby1_msgs_available=RBY1_MSGS_AVAILABLE,

            service_ready=readiness,

            command=command,
            command_stale=stale,
        )

    def shutdown_safely(
        self,
        turn_stream_off: bool = True,
    ) -> None:
        """Stop motion and optionally request Stream OFF."""

        self.stop(publish_immediately=True)

        for _ in range(4):
            self._publish_twist(
                VelocityCommand()
            )

        if (
            turn_stream_off
            and self.stream_enabled is True
        ):
            self.request_stream(False)
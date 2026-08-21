"""ROS 2 backend for the RB-Y1 control UI.

The backend keeps the Qt GUI independent from ROS message handling.

Supported paths:
- mobile base velocity through geometry_msgs/Twist
- power / servo / stream control through StateOnOff services
- robot state monitoring
- joint state monitoring
- joint position commands through Rby1JointCommand
- Cartesian pose monitoring through GetCartesianPose
- Cartesian position commands through Rby1CartesianCommand
- motion cancellation through cancel_control
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
import threading
import time
from typing import Deque, Dict, List, Optional, Tuple

from geometry_msgs.msg import Twist
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

# RB-Y1 전용 인터페이스가 없으면 cmd_vel 전용 모드로 동작한다.
try:
    from rby1_msgs.action import Rby1CartesianCommand, Rby1JointCommand
    from rby1_msgs.msg import CartesianCommand, JointCommand, RobotState
    from rby1_msgs.srv import GetCartesianPose, StateOnOff

    RBY1_MSGS_AVAILABLE = True
except ImportError:
    Rby1CartesianCommand = None  # type: ignore[assignment]
    Rby1JointCommand = None  # type: ignore[assignment]
    CartesianCommand = None  # type: ignore[assignment]
    JointCommand = None  # type: ignore[assignment]
    RobotState = None  # type: ignore[assignment]
    GetCartesianPose = None  # type: ignore[assignment]
    StateOnOff = None  # type: ignore[assignment]
    RBY1_MSGS_AVAILABLE = False

# RB-Y1 M v1.3 URDF 기준 관절 위치 제한값 (단위: rad).
JOINT_LIMITS_RAD = {
    "torso": (
        (-0.261799388, 0.261799388),
        (-0.523598776, 1.570796327),
        (-2.617993878, 1.570796327),
        (-0.785398163, 1.570796327),
        (-0.523598776, 0.523598776),
        (-2.35619449, 2.35619449),
    ),
    "right_arm": (
        (-3.141592654, 3.141592654),
        (-3.141592654, 0.017453293),
        (-3.141592654, 3.141592654),
        (-2.617993878, 0.017453293),
        (-3.141592654, 3.141592654),
        (-0.8726646260, 0.8726646260),
        (-1.5707963268, 1.5707963268),
    ),
    "left_arm": (
        (-3.141592654, 3.141592654),
        (-0.017453293, 3.141592654),
        (-3.141592654, 3.141592654),
        (-2.617993878, 0.017453293),
        (-3.141592654, 3.141592654),
        (-0.8726646260, 0.8726646260),
        (-1.5707963268, 1.5707963268),
    ),
    "head": ((-1.57, 1.57), (-1.57, 1.57)),
}


@dataclass(frozen=True)
class VelocityCommand:
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0

    @property
    def stopped(self) -> bool:
        return abs(self.vx) < 1e-9 and abs(self.vy) < 1e-9 and abs(self.wz) < 1e-9


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
        super().__init__("rby1_control_ui", namespace="rby1")

        # Topic과 Service 이름.
        self.declare_parameter("cmd_vel_topic", "cmd_vel")
        self.declare_parameter("robot_state_topic", "robot_state")
        self.declare_parameter("robot_power_service", "robot_power")
        self.declare_parameter("robot_servo_service", "robot_servo")
        self.declare_parameter("stream_control_service", "stream_control")

        # rby1_driver가 제공하는 관절/Cartesian 인터페이스.
        self.declare_parameter("joint_action", "robot_joint")
        self.declare_parameter("cartesian_action", "robot_cartesian")
        self.declare_parameter("cartesian_pose_service", "get_cartesian_pose")
        self.declare_parameter("cancel_control_service", "cancel_control")

        self.declare_parameter("right_arm_joint_state_topic", "joint_states/right_arm")
        self.declare_parameter("left_arm_joint_state_topic", "joint_states/left_arm")
        self.declare_parameter("torso_joint_state_topic", "joint_states/torso")
        self.declare_parameter("head_joint_state_topic", "joint_states/head")

        # Cartesian 명령에 사용할 기준 링크와 말단 링크.
        self.declare_parameter("right_cartesian_ref_link", "base")
        self.declare_parameter("right_cartesian_target_link", "ee_right")
        self.declare_parameter("left_cartesian_ref_link", "base")
        self.declare_parameter("left_cartesian_target_link", "ee_left")

        # 발행 주기, timeout 등 Backend 동작 설정.
        self.declare_parameter("use_rby1_services", True)
        self.declare_parameter("publish_rate_hz", 25.0)
        self.declare_parameter("command_timeout_sec", 0.35)
        self.declare_parameter("publish_zero_when_idle", True)
        self.declare_parameter("cartesian_state_period_sec", 0.25)
        self.declare_parameter("joint_jog_minimum_time_sec", 1.0)
        self.declare_parameter("cartesian_jog_minimum_time_sec", 1.0)

        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.robot_state_topic = str(self.get_parameter("robot_state_topic").value)
        self.robot_power_service = str(self.get_parameter("robot_power_service").value)
        self.robot_servo_service = str(self.get_parameter("robot_servo_service").value)
        self.stream_control_service = str(self.get_parameter("stream_control_service").value)

        self.joint_action_name = str(self.get_parameter("joint_action").value)
        self.cartesian_action_name = str(self.get_parameter("cartesian_action").value)
        self.cartesian_pose_service = str(self.get_parameter("cartesian_pose_service").value)
        self.cancel_control_service = str(self.get_parameter("cancel_control_service").value)

        self.joint_state_topics = {
            "right_arm": str(self.get_parameter("right_arm_joint_state_topic").value),
            "left_arm": str(self.get_parameter("left_arm_joint_state_topic").value),
            "torso": str(self.get_parameter("torso_joint_state_topic").value),
            "head": str(self.get_parameter("head_joint_state_topic").value),
        }

        self.cartesian_links = {
            "right_arm": (
                str(self.get_parameter("right_cartesian_ref_link").value),
                str(self.get_parameter("right_cartesian_target_link").value),
            ),
            "left_arm": (
                str(self.get_parameter("left_cartesian_ref_link").value),
                str(self.get_parameter("left_cartesian_target_link").value),
            ),
        }

        requested_services = bool(self.get_parameter("use_rby1_services").value)

        self.services_enabled = requested_services and RBY1_MSGS_AVAILABLE

        self.publish_rate_hz = self._positive_float(
            self.get_parameter("publish_rate_hz").value, fallback=25.0
        )

        self.command_timeout_sec = self._positive_float(
            self.get_parameter("command_timeout_sec").value, fallback=0.35
        )

        self.publish_zero_when_idle = bool(self.get_parameter("publish_zero_when_idle").value)

        self.cartesian_state_period_sec = self._positive_float(
            self.get_parameter("cartesian_state_period_sec").value, fallback=0.25
        )

        self.joint_jog_minimum_time_sec = self._positive_float(
            self.get_parameter("joint_jog_minimum_time_sec").value, fallback=1.0
        )

        self.cartesian_jog_minimum_time_sec = self._positive_float(
            self.get_parameter("cartesian_jog_minimum_time_sec").value, fallback=1.0
        )

        # UI와 ROS callback이 공유하는 명령 상태.
        self._lock = threading.RLock()

        self._command = VelocityCommand()
        self._last_command_update = time.monotonic()

        self._last_published = VelocityCommand()
        self._last_logged_command: Optional[VelocityCommand] = None

        self._events: Deque[Tuple[str, str]] = deque(maxlen=300)

        # /robot_state에서 받은 실제 로봇 상태.
        self.control_state: Optional[int] = None
        self.stream_enabled: Optional[bool] = None
        self.emo_active: Optional[bool] = None
        self.collision_active: Optional[bool] = None

        self._robot_state_received = False

        # Qt UI에 전달할 관절 및 Cartesian 상태 캐시.
        self._joint_groups_deg: Dict[str, Optional[List[float]]] = {
            "right_arm": None,
            "left_arm": None,
            "torso": None,
            "head": None,
        }

        self._cartesian_state: Dict[str, Optional[List[float]]] = {
            "right_arm": None,
            "left_arm": None,
        }

        self._cartesian_request_pending = {"right_arm": False, "left_arm": False}

        # Joint/Cartesian Action은 한 번에 하나만 실행한다.
        self._motion_busy = False
        self._active_motion_kind: Optional[str] = None
        self._active_goal_handle = None

        # Power ON -> Servo ON 준비 절차 상태 머신.
        self._prepare_stage = "idle"
        self._prepare_not_before = 0.0
        self._prepare_deadline = 0.0

        # 비동기 요청이 끝날 때까지 Future 참조를 유지한다.
        self._pending_futures: List[object] = []

        # 모바일 베이스 속도 Publisher.
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        # RB-Y1 Service, Action, Subscriber 객체.
        self.power_client = None # service: PowerOnOff
        self.servo_client = None # service: ServiceOnOff
        self.stream_client = None # service: StreamOnOff
        self.state_sub = None

        self.joint_action_client = None
        self.cartesian_action_client = None
        self.cartesian_pose_client = None # service: GetCartesianPose
        self.cancel_control_client = None # service: Trigger
        self.joint_state_subs = []

        if self.services_enabled:
            assert StateOnOff is not None
            assert RobotState is not None
            assert GetCartesianPose is not None
            assert Rby1JointCommand is not None
            assert Rby1CartesianCommand is not None

            self.power_client = self.create_client(StateOnOff, self.robot_power_service)

            self.servo_client = self.create_client(StateOnOff, self.robot_servo_service)

            self.stream_client = self.create_client(StateOnOff, self.stream_control_service)

            self.state_sub = self.create_subscription(
                RobotState, self.robot_state_topic, self._state_callback, 10
            )

            self.cartesian_pose_client = self.create_client(
                GetCartesianPose, self.cartesian_pose_service
            )

            self.cancel_control_client = self.create_client(Trigger, self.cancel_control_service)

            self.joint_action_client = ActionClient(self, Rby1JointCommand, self.joint_action_name)

            self.cartesian_action_client = ActionClient(
                self, Rby1CartesianCommand, self.cartesian_action_name
            )

            for group, topic in self.joint_state_topics.items():
                subscription = self.create_subscription(
                    JointState,
                    topic,
                    lambda msg, group_name=group: self._joint_state_callback(group_name, msg),
                    10,
                )
                self.joint_state_subs.append(subscription)

        elif requested_services:
            self._push_event("warning", "rby1_msgs를 불러오지 못해 cmd_vel 전용 모드로 시작합니다.")

        # 저장된 속도 명령을 일정 주기로 발행한다.
        self.publish_timer = self.create_timer(1.0 / self.publish_rate_hz, self._publish_cycle)

        # 로봇 준비 상태 머신을 주기적으로 진행한다.
        self.operation_timer = self.create_timer(0.05, self._process_prepare_operation)

        # Cartesian pose는 Service로 조회하고, Joint state는 Topic으로 받는다.
        self.cartesian_state_timer = self.create_timer(
            self.cartesian_state_period_sec, self._poll_cartesian_state
        )

        self._push_event(
            "info",
            f"ROS node started: "
            f"namespace={self.get_namespace()}, "
            f"cmd_vel={self.cmd_vel_pub.topic_name}",
        )

    # ==================================================================
    # 설정값 검증과 UI 이벤트
    # ==================================================================

    @staticmethod
    def _positive_float(value: object, fallback: float) -> float:
        try:
            result = float(value)
        except (TypeError, ValueError):
            return fallback

        if not math.isfinite(result) or result <= 0.0:
            return fallback

        return result

    def _push_event(self, level: str, message: str) -> None:
        timestamp = time.strftime("%H:%M:%S")

        with self._lock:
            self._events.append((level, f"[{timestamp}] {message}"))

        logger = self.get_logger()

        if level == "error":
            logger.error(message)
        elif level == "warning":
            logger.warning(message)
        else:
            logger.info(message)

    def drain_events(self) -> List[Tuple[str, str]]:
        with self._lock:
            items = list(self._events)
            self._events.clear()

        return items

    # ==================================================================
    # 상태 입력: RobotState / JointState Topic
    # ==================================================================

    def _state_callback(self, msg) -> None:
        """Receive the actual state published by the RB-Y1 driver."""

        self._robot_state_received = True

        new_control_state = int(msg.control_manager_state)
        new_stream_enabled = bool(msg.robot_stream_state)
        new_emo_active = bool(msg.emo_state)
        new_collision_active = bool(msg.collision)

        if new_control_state != self.control_state:
            self.control_state = new_control_state

            self._push_event("info", f"Robot control state changed to {new_control_state}.")

        if new_stream_enabled != self.stream_enabled:
            self.stream_enabled = new_stream_enabled

            self._push_event(
                "info", f"Robot stream state changed to {'ON' if new_stream_enabled else 'OFF'}."
            )
        # EMO 확인 +문제시 중단
        if new_emo_active != self.emo_active:
            self.emo_active = new_emo_active

            self._push_event(
                "warning" if new_emo_active else "info",
                f"EMO state changed to {'ACTIVE' if new_emo_active else 'RELEASED'}.",
            )

            if new_emo_active:
                self.stop(publish_immediately=True)
                self.cancel_motion()
                
        # 충돌 상태 확인 +문제시 중단   
        if new_collision_active != self.collision_active:
            self.collision_active = new_collision_active

            self._push_event(
                "warning" if new_collision_active else "info",
                f"Collision state changed to {'ACTIVE' if new_collision_active else 'CLEAR'}.",
            )

            if new_collision_active:
                self.stop(publish_immediately=True)
                self.cancel_motion()

    def _joint_state_callback(self, group: str, msg: JointState) -> None:
        """Cache a joint group state in operator-friendly degrees."""

        expected_names = {
            "right_arm": [f"right_arm_{index}" for index in range(7)],
            "left_arm": [f"left_arm_{index}" for index in range(7)],
            "torso": [f"torso_{index}" for index in range(6)],
            "head": [f"head_{index}" for index in range(2)],
        }

        names = expected_names.get(group)
        if names is None:
            return

        position_by_name = {
            str(name): float(position) for name, position in zip(msg.name, msg.position)
        }

        try:
            ordered_rad = [position_by_name[name] for name in names]
        except KeyError:
            # 예상한 관절 이름이 없을 때만 메시지 순서를 사용한다.
            if len(msg.position) < len(names):
                return
            ordered_rad = [float(value) for value in msg.position[: len(names)]]

        ordered_deg = [math.degrees(value) for value in ordered_rad]

        with self._lock:
            self._joint_groups_deg[group] = ordered_deg

    # ==================================================================
    # 상태 입력: Cartesian Pose Service
    # ==================================================================

    def _poll_cartesian_state(self) -> None:
        """Request current right/left end-effector poses asynchronously."""

        if (
            not self.services_enabled
            or self.cartesian_pose_client is None
            or GetCartesianPose is None
        ):
            return

        if not self.cartesian_pose_client.service_is_ready():
            return

        for arm in ("right_arm", "left_arm"):
            if self._cartesian_request_pending[arm]:
                continue

            ref_link, target_link = self.cartesian_links[arm]

            request = GetCartesianPose.Request()
            request.ref_link = ref_link
            request.target_link = target_link

            future = self.cartesian_pose_client.call_async(request)

            self._cartesian_request_pending[arm] = True
            self._pending_futures.append(future)

            future.add_done_callback(
                lambda done, arm_name=arm: self._cartesian_pose_done(done, arm_name)
            )

    def _cartesian_pose_done(self, future, arm: str) -> None:
        self._discard_future(future)
        self._cartesian_request_pending[arm] = False

        try:
            response = future.result()
        except Exception as exc:
            self._push_event("warning", f"Cartesian pose request failed for {arm}: {exc}")
            return

        if response is None:
            return

        transform = response.transform

        roll_deg, pitch_deg, yaw_deg = self._quaternion_to_rpy_deg(
            float(transform.rotation.x),
            float(transform.rotation.y),
            float(transform.rotation.z),
            float(transform.rotation.w),
        )

        pose = [
            float(transform.translation.x),
            float(transform.translation.y),
            float(transform.translation.z),
            roll_deg,
            pitch_deg,
            yaw_deg,
        ]

        with self._lock:
            self._cartesian_state[arm] = pose

    # ==================================================================
    # UI 상태 조회와 공통 동작 안전 검사
    # ==================================================================

    def get_motion_state(self) -> Dict[str, object]:
        """Return cached joint and Cartesian state for the Qt GUI."""

        with self._lock:
            joint_groups = {
                group: (list(values) if values is not None else None)
                for group, values in self._joint_groups_deg.items()
            }

            cartesian = {
                arm: (list(values) if values is not None else None)
                for arm, values in self._cartesian_state.items()
            }

        return {"joint_groups": joint_groups, "cartesian": cartesian}

    def _motion_command_allowed(self) -> bool:
        """Common safety gate before sending a manipulation goal."""

        if not self.services_enabled:
            self._push_event("warning", "RB-Y1 manipulation interfaces are disabled.")
            return False

        if self.emo_active is True:
            self._push_event("error", "Motion rejected: EMO is active.")
            return False

        if self.collision_active is True:
            self._push_event("error", "Motion rejected: collision state is active.")
            return False

        if self.control_state not in (2, 3):
            self._push_event(
                "warning", "Motion rejected: robot control state is not ENABLE/EXECUTING."
            )
            return False

        if self._motion_busy:
            self._push_event(
                "warning", "Motion rejected: another Joint/Cartesian command is still active."
            )
            return False

        return True

    # ==================================================================
    # 명령 출력: Joint Action
    # ==================================================================

    def jog_joint(self, group: str, index: int, delta_deg: float) -> None:
        """Jog one joint by sending a full-group absolute target."""

        # 현재 관절값 기반 목표값 생성.
        with self._lock:
            current = self._joint_groups_deg.get(group)
            current_copy = list(current) if current is not None else None

        # 현재 상태 및 관절 번호 유효성 검사.
        if current_copy is None:
            self._push_event("warning", f"Joint jog rejected: no state for {group}.")
            return

        if index < 0 or index >= len(current_copy):
            self._push_event("error", f"Joint jog rejected: invalid index {index}.")
            return

        # delta_deg 반영
        current_copy[index] += float(delta_deg)

        self.move_joint_group(group, current_copy, self.joint_jog_minimum_time_sec)

    def move_joint_group(self, group: str, targets_deg: List[float], minimum_time: float) -> None:
        """Send an absolute joint-position command.
        GUI values are degrees.  The ROS action receives radians.
        """
        expected_dof = {"right_arm": 7, "left_arm": 7, "torso": 6, "head": 2}

        dof = expected_dof.get(group)
        if dof is None:
            self._push_event("error", f"Unknown joint group: {group}.")
            return

        if len(targets_deg) != dof:
            self._push_event("error", f"{group} expects {dof} targets, got {len(targets_deg)}.")
            return

        values_deg = [float(value) for value in targets_deg]

        # inf, NaN 검사
        if not all(math.isfinite(v) for v in values_deg):
            self._push_event("error", "Non-finite joint target rejected.")
            return

        # 각 관절별 제한값 검사
        if not self._validate_joint_targets(group, values_deg):
            return

        if not self._motion_command_allowed():
            return

        if self.joint_action_client is None or Rby1JointCommand is None or JointCommand is None:
            self._push_event("error", "Joint action client is unavailable.")
            return

        if not self.joint_action_client.server_is_ready():
            self._push_event("warning", f"Joint action server not ready: {self.joint_action_name}")
            return

        command = JointCommand()
        command.position = [math.radians(value) for value in values_deg]
        command.minimum_time = max(0.1, float(minimum_time))

        goal = Rby1JointCommand.Goal()
        # 선택한 관절 그룹의 명령을 Action Goal에 설정
        setattr(goal, group, command)

        self._send_motion_goal(client=self.joint_action_client, goal=goal, label=f"Joint {group}")

    def _validate_joint_targets(self, group: str, targets_deg: List[float]) -> bool:
        """Check requested joint targets against RB-Y1 M v1.3 limits."""

        limits = JOINT_LIMITS_RAD.get(group)

        if limits is None:
            self._push_event("error", f"No joint limits defined for group: {group}.")
            return False

        if len(targets_deg) != len(limits):
            self._push_event("error", f"{group} target count does not match joint limits.")
            return False

        for index, (target_deg, limit) in enumerate(zip(targets_deg, limits)):
            lower_rad, upper_rad = limit
            target_rad = math.radians(float(target_deg))

            if target_rad < lower_rad or target_rad > upper_rad:
                lower_deg = math.degrees(lower_rad)
                upper_deg = math.degrees(upper_rad)

                joint_name = f"{group}_{index}"

                self._push_event(
                    "warning",
                    "Motion rejected: "
                    f"{joint_name} target "
                    f"{target_deg:+.2f} deg is outside "
                    f"[{lower_deg:+.2f}, "
                    f"{upper_deg:+.2f}] deg.",
                )

                return False

        return True

    # ==================================================================
    # 명령 출력: Cartesian Action
    # ==================================================================

    def jog_cartesian(
        self, arm: str, axis_index: int, delta: float, reference_frame: str = "base"
    ) -> None:
        """Jog one Cartesian component from the latest absolute pose."""

        del reference_frame

        with self._lock:
            current = self._cartesian_state.get(arm)
            target = list(current) if current is not None else None

        if target is None:
            self._push_event("warning", f"Cartesian jog rejected: no current pose for {arm}.")
            return

        if axis_index < 0 or axis_index >= 6:
            self._push_event("error", f"Cartesian jog rejected: invalid axis {axis_index}.")
            return

        target[axis_index] += float(delta)

        self.move_cartesian(
            arm, target, self.cartesian_jog_minimum_time_sec, reference_frame="base"
        )

    def move_cartesian(
        self, arm: str, target: List[float], minimum_time: float, reference_frame: str = "base"
    ) -> None:
        """Send an absolute Cartesian target.

        target = [x, y, z, roll_deg, pitch_deg, yaw_deg]
        """

        if arm not in ("right_arm", "left_arm"):
            self._push_event("error", f"Unknown Cartesian arm: {arm}.")
            return

        if len(target) != 6:
            self._push_event("error", "Cartesian target must contain 6 values.")
            return

        values = [float(value) for value in target]

        if not all(math.isfinite(v) for v in values):
            self._push_event("error", "Non-finite Cartesian target rejected.")
            return

        if not self._motion_command_allowed():
            return

        if (
            self.cartesian_action_client is None
            or Rby1CartesianCommand is None
            or CartesianCommand is None
        ):
            self._push_event("error", "Cartesian action client is unavailable.")
            return

        if not self.cartesian_action_client.server_is_ready():
            self._push_event(
                "warning", f"Cartesian action server not ready: {self.cartesian_action_name}"
            )
            return

        configured_ref, target_link = self.cartesian_links[arm]

        # UI에서 Base를 선택하면 설정된 기준 링크를 사용한다.
        ref_link = configured_ref if reference_frame == "base" else str(reference_frame)

        command = CartesianCommand()
        command.ref_link = ref_link
        command.target_link = target_link

        command.transform.translation.x = values[0]
        command.transform.translation.y = values[1]
        command.transform.translation.z = values[2]

        qx, qy, qz, qw = self._rpy_deg_to_quaternion(values[3], values[4], values[5])

        command.transform.rotation.x = qx
        command.transform.rotation.y = qy
        command.transform.rotation.z = qz
        command.transform.rotation.w = qw

        command.minimum_time = max(0.1, float(minimum_time))

        goal = Rby1CartesianCommand.Goal()
        setattr(goal, arm, command)

        self._send_motion_goal(
            client=self.cartesian_action_client, goal=goal, label=f"Cartesian {arm}"
        )

    # Joint/Cartesian Action이 함께 사용하는 비동기 처리.
    def _send_motion_goal(self, client, goal, label: str) -> None:
        self._motion_busy = True
        self._active_motion_kind = label

        future = client.send_goal_async(goal)
        self._pending_futures.append(future)

        # action 서버가 goal을 수락/거부하면 호출
        future.add_done_callback(
            lambda done, motion_label=label: self._motion_goal_response(done, motion_label)
        )

        self._push_event("info", f"{label} goal requested.")

    def _motion_goal_response(self, future, label: str) -> None:
        self._discard_future(future)

        try:
            goal_handle = future.result()
        except Exception as exc:
            self._motion_busy = False
            self._active_motion_kind = None
            self._active_goal_handle = None

            self._push_event("error", f"{label} goal request failed: {exc}")
            return

        # goal이 거부되거나 None
        if goal_handle is None or not goal_handle.accepted:
            self._motion_busy = False
            self._active_motion_kind = None
            self._active_goal_handle = None

            self._push_event("error", f"{label} goal was rejected.")
            return

        self._active_goal_handle = goal_handle

        self._push_event("info", f"{label} goal accepted.")

        result_future = goal_handle.get_result_async()
        self._pending_futures.append(result_future)

        result_future.add_done_callback(
            lambda done, motion_label=label: self._motion_result_done(done, motion_label)
        )

    def _motion_result_done(self, future, label: str) -> None:
        self._discard_future(future)

        self._motion_busy = False
        self._active_motion_kind = None
        self._active_goal_handle = None

        try:
            wrapped_result = future.result()
            result = wrapped_result.result
        except Exception as exc:
            self._push_event("error", f"{label} result failed: {exc}")
            return

        if bool(result.success):
            self._push_event("info", f"{label} completed: {result.finish_code}")
        else:
            self._push_event("warning", f"{label} finished unsuccessfully: {result.finish_code}")

    # ==================================================================
    # 명령 출력: Cancel Service
    # ==================================================================

    def cancel_motion(self) -> None:
        """Cancel robot control through the driver's Trigger service.

        The driver's cancel_control service is intentionally used here as
        the safety-oriented common cancel path for Joint/Cartesian control.
        """

        if self.cancel_control_client is None:
            return

        if not self.cancel_control_client.service_is_ready():
            self._push_event("warning", f"Cancel service not ready: {self.cancel_control_service}")
            return

        request = Trigger.Request()
        future = self.cancel_control_client.call_async(request)

        self._pending_futures.append(future)

        future.add_done_callback(self._cancel_motion_done)

        self._push_event("warning", "Motion cancel requested.")

    def _cancel_motion_done(self, future) -> None:
        self._discard_future(future)

        try:
            result = future.result()
        except Exception as exc:
            self._push_event("error", f"Motion cancel failed: {exc}")
            return

        if result is not None and bool(result.success):
            self._motion_busy = False
            self._active_motion_kind = None
            self._active_goal_handle = None

            self._push_event("info", f"Motion cancel succeeded: {result.message}")
        else:
            message = (
                getattr(result, "message", "No response") if result is not None else "No response"
            )
            self._push_event("error", f"Motion cancel failed: {message}")

    # ==================================================================
    # 명령 출력: 모바일 베이스 Twist Topic
    # ==================================================================

    def set_velocity(self, vx: float, vy: float, wz: float) -> None:
        """Update the target mobile-base velocity."""

        values = (float(vx), float(vy), float(wz))

        if not all(math.isfinite(value) for value in values):
            self._push_event("error", "Invalid non-finite velocity command rejected.")
            return

        with self._lock:
            self._command = VelocityCommand(*values)
            self._last_command_update = time.monotonic()

    def stop(self, publish_immediately: bool = True) -> None:
        """Set the velocity command to zero."""

        with self._lock:
            self._command = VelocityCommand()
            self._last_command_update = time.monotonic()

        if publish_immediately:
            self._publish_twist(VelocityCommand())

    def _current_command(self) -> Tuple[VelocityCommand, bool]:
        with self._lock:
            command = self._command
            age = time.monotonic() - self._last_command_update

        stale = age > self.command_timeout_sec

        return command, stale

    def _publish_cycle(self) -> None:
        """Publish Twist at the configured fixed rate."""

        command, stale = self._current_command()

        if stale:
            command = VelocityCommand()

        if self.publish_zero_when_idle or not command.stopped or not self._last_published.stopped:
            self._publish_twist(command)

    def _publish_twist(self, command: VelocityCommand) -> None:
        """Convert VelocityCommand to geometry_msgs/Twist."""

        msg = Twist()

        msg.linear.x = command.vx
        msg.linear.y = command.vy
        msg.angular.z = command.wz

        self.cmd_vel_pub.publish(msg)

        self._last_published = command

        # 같은 명령이 반복 발행돼도 로그는 값이 바뀔 때만 남긴다.
        if command != self._last_logged_command:
            self._last_logged_command = command

            self._push_event(
                "info", f"cmd_vel: vx={command.vx:+.3f}, vy={command.vy:+.3f}, wz={command.wz:+.3f}"
            )

    # ==================================================================
    # 명령 출력: Power / Servo / Stream Service
    # ==================================================================

    def prepare_robot(self) -> None:
        """Run Power ON -> Servo ON as a non-blocking sequence."""

        if not self.services_enabled:
            self._push_event("warning", "RB-Y1 service control is disabled.")
            return

        if self._prepare_stage != "idle":
            self._push_event("warning", "Robot preparation is already running.")
            return

        if self.control_state in (2, 3):
            self._push_event("info", "Robot is already powered and servo-enabled.")
            return

        self._prepare_stage = "power_request"
        self._prepare_deadline = time.monotonic() + 20.0

        self._push_event("info", "Starting robot preparation: Power ON -> Servo ON.")

    def request_power(self, enabled: bool, target: str = "all") -> None:
        """Request robot power ON or OFF."""

        if not enabled:
            self.stop(publish_immediately=True)

        self._request_state_on_off(
            client=self.power_client, enabled=enabled, parameters=target, value=0.0, label="Power"
        )

    def request_servo(self, enabled: bool, target: str = "all") -> None:
        """Request robot servo ON or OFF."""

        if not enabled:
            self.stop(publish_immediately=True)

        self._request_state_on_off(
            client=self.servo_client, enabled=enabled, parameters=target, value=0.0, label="Servo"
        )

    def request_stream(self, enabled: bool, value: float = 0.0) -> None:
        """Request cmd_vel stream control ON or OFF."""

        if not enabled:
            self.stop(publish_immediately=True)

        self._request_state_on_off(
            client=self.stream_client, enabled=enabled, parameters="", value=value, label="Stream"
        )

    def _request_state_on_off(
        self, client, enabled: bool, parameters: str, value: float, label: str
    ) -> None:
        """Common StateOnOff service request function."""

        if not self.services_enabled or client is None:
            self._push_event("warning", f"{label} service is unavailable.")
            return

        if not client.service_is_ready():
            self._push_event("warning", f"{label} service not ready: {client.srv_name}")
            return

        assert StateOnOff is not None

        request = StateOnOff.Request()

        request.state = bool(enabled)
        request.parameters = str(parameters)
        request.value = float(value)

        future = client.call_async(request)

        self._pending_futures.append(future)

        future.add_done_callback(
            lambda done, service_label=label, requested=bool(enabled): self._state_on_off_done(
                done, service_label, requested
            )
        )

        self._push_event("info", f"{label} {'ON' if enabled else 'OFF'} requested.")

    def _state_on_off_done(self, future, label: str, enabled: bool) -> None:
        """Process Power, Servo, or Stream service response."""

        self._discard_future(future)

        try:
            result = future.result()
        except Exception as exc:
            self._push_event("error", f"{label} service call failed: {exc}")
            return

        if result is not None and bool(result.success):
            # Stream 상태는 /robot_state를 우선하고, 미수신 시 응답값을 사용한다.
            if label == "Stream" and not self._robot_state_received:
                self.stream_enabled = enabled

            self._push_event("info", f"{label} {'ON' if enabled else 'OFF'} succeeded.")
            return

        message = getattr(result, "message", "No response") if result is not None else "No response"

        self._push_event("error", f"{label} request failed: {message}")

    def _process_prepare_operation(self) -> None:
        """Process the Power -> Servo preparation state machine."""

        if self._prepare_stage == "idle":
            return

        now = time.monotonic()

        if now > self._prepare_deadline:
            self._push_event("error", "Robot preparation timed out.")

            self._prepare_stage = "idle"
            return

        if self._prepare_stage == "power_request":
            if self.power_client is None or not self.power_client.service_is_ready():
                return

            self._call_state_service(
                self.power_client, state=True, parameters="all", callback=self._power_done
            )

            self._prepare_stage = "power_pending"

            self._push_event("info", "Power ON request sent.")

            return

        if self._prepare_stage == "power_wait":
            if now >= self._prepare_not_before:
                self._prepare_stage = "servo_request"

            return

        if self._prepare_stage == "servo_request":
            if self.servo_client is None or not self.servo_client.service_is_ready():
                return

            self._call_state_service(
                self.servo_client, state=True, parameters="all", callback=self._servo_done
            )

            self._prepare_stage = "servo_pending"

            self._push_event("info", "Servo ON request sent.")

            return

        if self._prepare_stage == "wait_state":
            if self.control_state in (2, 3):
                self._push_event("info", "Robot preparation completed.")

                self._prepare_stage = "idle"

            elif self.control_state is None and now >= self._prepare_not_before:
                self._push_event(
                    "warning",
                    "Servo request succeeded, but "
                    "robot_state is unavailable; "
                    "continuing without state confirmation.",
                )

                self._prepare_stage = "idle"

    def _call_state_service(self, client, state: bool, parameters: str, callback) -> None:
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

        result = self._safe_service_result(future, "Power ON")

        if result:
            self._prepare_not_before = time.monotonic() + 1.0

            self._prepare_stage = "power_wait"
        else:
            self._prepare_stage = "idle"

    def _servo_done(self, future) -> None:
        self._discard_future(future)

        result = self._safe_service_result(future, "Servo ON")

        if result:
            self._prepare_not_before = time.monotonic() + 1.0

            self._prepare_stage = "wait_state"
        else:
            self._prepare_stage = "idle"

    def _safe_service_result(self, future, name: str) -> bool:
        try:
            result = future.result()
        except Exception as exc:
            self._push_event("error", f"{name} service failed: {exc}")

            return False

        if result is not None and bool(result.success):
            self._push_event("info", f"{name} succeeded.")

            return True

        message = getattr(result, "message", "No response") if result is not None else "No response"

        self._push_event("error", f"{name} failed: {message}")

        return False

    def _discard_future(self, future) -> None:
        try:
            self._pending_futures.remove(future)
        except ValueError:
            pass

    # ==================================================================
    # UI Snapshot과 안전 종료
    # ==================================================================

    def snapshot(self) -> BackendSnapshot:
        """Return a thread-safe state snapshot for the Qt UI."""

        command, stale = self._current_command()

        readiness = {
            "power": bool(self.power_client and self.power_client.service_is_ready()),
            "servo": bool(self.servo_client and self.servo_client.service_is_ready()),
            "stream": bool(self.stream_client and self.stream_client.service_is_ready()),
            "cartesian_pose": bool(
                self.cartesian_pose_client and self.cartesian_pose_client.service_is_ready()
            ),
            "cancel_control": bool(
                self.cancel_control_client and self.cancel_control_client.service_is_ready()
            ),
            "joint_action": bool(
                self.joint_action_client and self.joint_action_client.server_is_ready()
            ),
            "cartesian_action": bool(
                self.cartesian_action_client and self.cartesian_action_client.server_is_ready()
            ),
        }

        return BackendSnapshot(
            namespace=self.get_namespace(),
            cmd_vel_topic=self.cmd_vel_pub.topic_name,
            cmd_vel_subscribers=self.count_subscribers(self.cmd_vel_pub.topic_name),
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

    def shutdown_safely(self, turn_stream_off: bool = True) -> None:
        """Stop motion and optionally request Stream OFF."""

        self.stop(publish_immediately=True)
        self.cancel_motion()

        for _ in range(4):
            self._publish_twist(VelocityCommand())

        if turn_stream_off and self.stream_enabled is True:
            self.request_stream(False)

    # ==================================================================  
    # 자세 표현 변환 유틸리티.
    # ==================================================================

    @staticmethod
    def _quaternion_to_rpy_deg(
        x: float, y: float, z: float, w: float
    ) -> Tuple[float, float, float]:
        """Quaternion -> roll/pitch/yaw in degrees."""

        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * y - z * x)
        sinp = max(-1.0, min(1.0, sinp))
        pitch = math.asin(sinp)

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))

    @staticmethod
    def _rpy_deg_to_quaternion(
        roll_deg: float, pitch_deg: float, yaw_deg: float
    ) -> Tuple[float, float, float, float]:
        """Roll/pitch/yaw in degrees -> quaternion x/y/z/w."""

        roll = math.radians(roll_deg)
        pitch = math.radians(pitch_deg)
        yaw = math.radians(yaw_deg)

        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy

        return x, y, z, w
"""ROS 2 bridge for the RBY1 MuJoCo and physical grippers."""

from __future__ import annotations

import math
import time
from typing import Optional, Sequence

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray
from std_srvs.srv import SetBool, Trigger

from .core import Calibration, GripperDriver, GripperError, GripperState
from .sdk_bus import create_sdk_bus
from .sim_rpc import MuJoCoGripperDriver


def _two_values(values: Sequence, label: str):
    if len(values) != 2:
        raise ValueError(f'{label} must contain exactly 2 values')
    return values


class GripperBridgeNode(Node):
    """Expose one normalized ROS interface for simulation and hardware."""

    def __init__(self) -> None:
        super().__init__('gripper_bridge')

        # Backend selection and official MuJoCo gRPC endpoint.
        self.declare_parameter('backend', 'mujoco')
        self.declare_parameter('robot_address', '127.0.0.1:50051')
        self.declare_parameter('rpc_timeout_sec', 2.0)
        self.declare_parameter('mujoco_velocity_percent', 100)
        self.declare_parameter('mujoco_force_percent', 50)
        self.declare_parameter('initialize_grippers_on_start', True)

        # Physical RBY1 Dynamixel gripper settings.
        self.declare_parameter('device_name', '')
        self.declare_parameter('baud_rate', 2_000_000)
        self.declare_parameter('right_motor_id', 0)
        self.declare_parameter('left_motor_id', 1)
        self.declare_parameter('torque_constants', [1.0, 1.0])
        self.declare_parameter('position_torque_limit', 5.0)
        self.declare_parameter('closed_at_minimum', [True, True])
        self.declare_parameter('endpoint_margin_ratio', 0.0)
        self.declare_parameter('minimum_calibration_span_rad', 0.01)

        self.declare_parameter('use_saved_calibration', False)
        self.declare_parameter('calibration_min_rad', [0.0, 0.0])
        self.declare_parameter('calibration_max_rad', [0.0, 0.0])
        self.declare_parameter('auto_home', False)
        self.declare_parameter('auto_enable_torque', True)
        self.declare_parameter('disable_torque_on_shutdown', True)

        self.declare_parameter('homing_torque', 0.3)
        self.declare_parameter('homing_sample_period_sec', 0.1)
        self.declare_parameter('homing_stall_threshold_rad', 0.000001)
        self.declare_parameter('homing_stall_samples', 30)
        self.declare_parameter('homing_direction_timeout_sec', 15.0)
        self.declare_parameter('homing_max_read_failures', 3)

        self.declare_parameter('control_rate_hz', 10.0)
        self.declare_parameter('state_rate_hz', 10.0)

        self._backend = str(
            self.get_parameter('backend').value
        ).strip().lower()
        if self._backend not in ('mujoco', 'dynamixel'):
            raise ValueError('backend must be "mujoco" or "dynamixel"')

        self._motor_ids = (
            int(self.get_parameter('right_motor_id').value),
            int(self.get_parameter('left_motor_id').value),
        )
        self._closed_at_minimum = tuple(
            bool(value)
            for value in _two_values(
                self.get_parameter('closed_at_minimum').value,
                'closed_at_minimum',
            )
        )
        self._endpoint_margin_ratio = float(
            self.get_parameter('endpoint_margin_ratio').value
        )
        self._minimum_calibration_span_rad = float(
            self.get_parameter('minimum_calibration_span_rad').value
        )
        self._disable_torque_on_shutdown = bool(
            self.get_parameter('disable_torque_on_shutdown').value
        )
        self._last_warning_at = {}
        self._last_ready: Optional[bool] = None
        self._last_state: Optional[GripperState] = None
        self._closed = False
        self._sdk_version = 'not-used'

        if self._backend == 'mujoco':
            address = str(self.get_parameter('robot_address').value)
            self._driver = MuJoCoGripperDriver(
                address,
                velocity_percent=int(
                    self.get_parameter('mujoco_velocity_percent').value
                ),
                force_percent=int(
                    self.get_parameter('mujoco_force_percent').value
                ),
                timeout_sec=float(
                    self.get_parameter('rpc_timeout_sec').value
                ),
            )
            self._hardware_id = f'mujoco://{address}'
            if bool(
                self.get_parameter('initialize_grippers_on_start').value
            ):
                self._driver.initialize()
        else:
            torque_constants = _two_values(
                self.get_parameter('torque_constants').value,
                'torque_constants',
            )
            requested_device = str(
                self.get_parameter('device_name').value
            )
            bus, device_name, self._sdk_version = create_sdk_bus(
                requested_device,
                warning=self.get_logger().warning,
            )
            self._hardware_id = device_name
            self._driver = GripperDriver(
                bus,
                motor_ids=self._motor_ids,
                torque_constants=torque_constants,
                baud_rate=int(self.get_parameter('baud_rate').value),
                position_torque_limit=float(
                    self.get_parameter('position_torque_limit').value
                ),
            )
            self._driver.initialize()
            self._configure_dynamixel_startup()

        ready_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._state_pub = self.create_publisher(
            Float64MultiArray, 'gripper/state', 10
        )
        self._motor_state_pub = self.create_publisher(
            JointState, 'gripper/motor_state', 10
        )
        self._ready_pub = self.create_publisher(
            Bool, 'gripper/ready', ready_qos
        )
        self._diagnostics_pub = self.create_publisher(
            DiagnosticArray, 'gripper/diagnostics', 10
        )
        self._command_sub = self.create_subscription(
            Float64MultiArray,
            'gripper/command',
            self._on_command,
            10,
        )
        self._home_service = self.create_service(
            Trigger, 'gripper/home', self._on_home
        )
        self._torque_service = self.create_service(
            SetBool, 'gripper/torque_enable', self._on_torque_enable
        )

        state_rate = float(self.get_parameter('state_rate_hz').value)
        if not math.isfinite(state_rate) or state_rate <= 0.0:
            raise ValueError('state_rate_hz must be positive')
        self._state_timer = self.create_timer(
            1.0 / state_rate, self._publish_state
        )

        self._control_timer = None
        if self._backend == 'dynamixel':
            control_rate = float(
                self.get_parameter('control_rate_hz').value
            )
            if not math.isfinite(control_rate) or control_rate <= 0.0:
                raise ValueError('control_rate_hz must be positive')
            self._control_timer = self.create_timer(
                1.0 / control_rate, self._refresh_target
            )

        self._publish_ready(force=True)
        self._publish_diagnostics()
        if self._backend == 'mujoco':
            self.get_logger().info(
                'Gripper bridge connected to the MuJoCo '
                f'GripperCommandService at {self._driver.address}. '
                'Command order is [right, left], where '
                '0.0=open and 1.0=closed.'
            )
        else:
            self.get_logger().info(
                'Gripper bridge initialized '
                f'(Dynamixel device={self._hardware_id}, '
                f'IDs={self._motor_ids}, rby1_sdk={self._sdk_version}). '
                'Command order is [right, left], where '
                '0.0=open and 1.0=closed.'
            )
            if self._driver.calibration is None:
                self.get_logger().warning(
                    'Physical gripper is not calibrated. Call gripper/home '
                    'before sending commands, or configure saved calibration.'
                )

    def _configure_dynamixel_startup(self) -> None:
        if bool(self.get_parameter('use_saved_calibration').value):
            calibration = self._calibration_from_parameters()
            self._driver.load_calibration(calibration)
            self.get_logger().info(
                'Loaded saved gripper calibration: '
                f'min={list(calibration.minimum_rad)}, '
                f'max={list(calibration.maximum_rad)}'
            )

        if bool(self.get_parameter('auto_home').value):
            self.get_logger().warning(
                'auto_home is enabled; both physical grippers will traverse '
                'their full range during startup'
            )
            self._home_driver()
        elif (
            self._driver.calibration is not None
            and bool(self.get_parameter('auto_enable_torque').value)
        ):
            self._driver.set_torque_enabled(True)

    def _calibration_from_parameters(self) -> Calibration:
        minimum = _two_values(
            self.get_parameter('calibration_min_rad').value,
            'calibration_min_rad',
        )
        maximum = _two_values(
            self.get_parameter('calibration_max_rad').value,
            'calibration_max_rad',
        )
        return Calibration(
            minimum_rad=(float(minimum[0]), float(minimum[1])),
            maximum_rad=(float(maximum[0]), float(maximum[1])),
            closed_at_minimum=self._closed_at_minimum,
            endpoint_margin_ratio=self._endpoint_margin_ratio,
            minimum_span_rad=self._minimum_calibration_span_rad,
        )

    def _home_driver(self) -> Optional[Calibration]:
        if self._backend == 'mujoco':
            self._driver.home()
            self.get_logger().info(
                'MuJoCo right and left grippers initialized through gRPC'
            )
            return None

        calibration = self._driver.home(
            homing_torque=float(self.get_parameter('homing_torque').value),
            sample_period_sec=float(
                self.get_parameter('homing_sample_period_sec').value
            ),
            stall_threshold_rad=float(
                self.get_parameter('homing_stall_threshold_rad').value
            ),
            stall_samples=int(
                self.get_parameter('homing_stall_samples').value
            ),
            direction_timeout_sec=float(
                self.get_parameter('homing_direction_timeout_sec').value
            ),
            max_read_failures=int(
                self.get_parameter('homing_max_read_failures').value
            ),
            closed_at_minimum=self._closed_at_minimum,
            endpoint_margin_ratio=self._endpoint_margin_ratio,
            minimum_span_rad=self._minimum_calibration_span_rad,
        )
        self.get_logger().info(
            'Physical gripper homing complete. Save these values to skip '
            f'future homing: min={list(calibration.minimum_rad)}, '
            f'max={list(calibration.maximum_rad)}'
        )
        return calibration

    def _on_command(self, message: Float64MultiArray) -> None:
        if len(message.data) != 2:
            self._warn_throttled(
                'bad_command_length',
                'Rejected gripper command: data must be [right, left]',
            )
            return
        try:
            self._driver.command(message.data)
        except GripperError as exc:
            self._warn_throttled(
                'command_rejected', f'Rejected gripper command: {exc}'
            )
        self._publish_ready()

    def _on_home(self, _request, response):
        if self._backend == 'mujoco':
            self.get_logger().warning(
                'MuJoCo gripper re-initialization requested'
            )
        else:
            self.get_logger().warning(
                'Homing requested; keep both physical grippers clear while '
                'they traverse their full range'
            )
        not_ready = Bool()
        not_ready.data = False
        self._ready_pub.publish(not_ready)
        self._last_ready = False
        try:
            calibration = self._home_driver()
            response.success = True
            if calibration is None:
                response.message = 'MuJoCo grippers initialized'
            else:
                response.message = (
                    f'min={list(calibration.minimum_rad)}, '
                    f'max={list(calibration.maximum_rad)}'
                )
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            self.get_logger().error(f'Gripper initialization failed: {exc}')
        self._publish_ready(force=True)
        self._publish_diagnostics()
        return response

    def _on_torque_enable(self, request, response):
        try:
            self._driver.set_torque_enabled(bool(request.data))
            response.success = True
            if self._backend == 'mujoco':
                response.message = 'MuJoCo gripper RPC initialized'
            else:
                response.message = (
                    'gripper torque enabled'
                    if request.data
                    else 'gripper torque disabled'
                )
        except GripperError as exc:
            response.success = False
            response.message = str(exc)
        self._publish_ready(force=True)
        self._publish_diagnostics()
        return response

    def _refresh_target(self) -> None:
        try:
            self._driver.repeat_last_command()
        except GripperError as exc:
            self._warn_throttled(
                'target_refresh', f'Failed to refresh gripper target: {exc}'
            )

    def _publish_state(self) -> None:
        try:
            state = self._driver.read_state()
        except GripperError as exc:
            self._warn_throttled(
                'state_read', f'Failed to read gripper state: {exc}'
            )
            self._publish_ready()
            self._publish_diagnostics()
            return

        self._last_state = state
        normalized = Float64MultiArray()
        normalized.data = list(state.close_ratios)
        self._state_pub.publish(normalized)

        if self._backend == 'dynamixel':
            motor_state = JointState()
            motor_state.header.stamp = self.get_clock().now().to_msg()
            motor_state.name = [
                'right_gripper_motor',
                'left_gripper_motor',
            ]
            motor_state.position = list(state.positions_rad)
            motor_state.velocity = list(state.velocities_rad_s)
            self._motor_state_pub.publish(motor_state)

        self._publish_ready()
        self._publish_diagnostics(state)

    def _publish_ready(self, force: bool = False) -> None:
        ready = self._driver.ready
        if force or ready != self._last_ready:
            message = Bool()
            message.data = ready
            self._ready_pub.publish(message)
            self._last_ready = ready

    def _publish_diagnostics(
        self, state: Optional[GripperState] = None
    ) -> None:
        state = state or self._last_state
        status = DiagnosticStatus()
        namespace = self.get_namespace().rstrip('/')
        status.name = (
            f'{namespace}/gripper_bridge'
            if namespace
            else '/gripper_bridge'
        )
        status.hardware_id = self._hardware_id

        if self._backend == 'mujoco':
            if not self._driver.healthy:
                status.level = DiagnosticStatus.ERROR
                status.message = 'MuJoCo gripper RPC is unhealthy'
            elif self._driver.busy:
                status.level = DiagnosticStatus.WARN
                status.message = 'MuJoCo gripper initialization in progress'
            elif not self._driver.initialized:
                status.level = DiagnosticStatus.WARN
                status.message = 'MuJoCo gripper initialization required'
            else:
                status.level = DiagnosticStatus.OK
                status.message = 'MuJoCo gripper RPC ready'
            values = {
                'backend': self._backend,
                'ready': str(self._driver.ready).lower(),
                'initialized': str(self._driver.initialized).lower(),
                'healthy': str(self._driver.healthy).lower(),
                'robot_address': self._driver.address,
                'state_source': 'last_rpc_accepted_target',
                'velocity_percent': str(
                    self._driver.velocity_percent
                ),
                'force_percent': str(self._driver.force_percent),
            }
        else:
            if not self._driver.healthy:
                status.level = DiagnosticStatus.ERROR
                status.message = 'Dynamixel communication is unhealthy'
            elif self._driver.busy:
                status.level = DiagnosticStatus.WARN
                status.message = 'Homing in progress'
            elif self._driver.calibration is None:
                status.level = DiagnosticStatus.WARN
                status.message = 'Calibration required'
            elif not self._driver.enabled:
                status.level = DiagnosticStatus.WARN
                status.message = 'Torque disabled'
            else:
                status.level = DiagnosticStatus.OK
                status.message = 'Ready'

            values = {
                'backend': self._backend,
                'ready': str(self._driver.ready).lower(),
                'initialized': str(self._driver.initialized).lower(),
                'healthy': str(self._driver.healthy).lower(),
                'torque_enabled': str(self._driver.enabled).lower(),
                'motor_ids_right_left': str(list(self._motor_ids)),
                'state_source': 'dynamixel_measurement',
            }
            calibration = self._driver.calibration
            if calibration is not None:
                values['calibration_min_rad_right_left'] = str(
                    list(calibration.minimum_rad)
                )
                values['calibration_max_rad_right_left'] = str(
                    list(calibration.maximum_rad)
                )

        if self._driver.target_close_ratios is not None:
            values['target_close_ratio_right_left'] = str(
                list(self._driver.target_close_ratios)
            )
        if state is not None:
            values['close_ratio_right_left'] = str(
                list(state.close_ratios)
            )
            if self._backend == 'dynamixel':
                values['position_rad_right_left'] = str(
                    list(state.positions_rad)
                )
                values['current_amp_right_left'] = str(
                    list(state.currents_amp)
                )
                values['temperature_c_right_left'] = str(
                    list(state.temperatures_c)
                )
        status.values = [
            KeyValue(key=key, value=value)
            for key, value in values.items()
        ]

        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [status]
        self._diagnostics_pub.publish(message)

    def _warn_throttled(
        self, key: str, message: str, period_sec: float = 2.0
    ) -> None:
        now = time.monotonic()
        last = self._last_warning_at.get(key, -math.inf)
        if now - last >= period_sec:
            self.get_logger().warning(message)
            self._last_warning_at[key] = now

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._driver.shutdown(self._disable_torque_on_shutdown)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = GripperBridgeNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

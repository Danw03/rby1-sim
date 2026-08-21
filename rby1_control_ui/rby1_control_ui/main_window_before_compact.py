"""Day 1 Qt shell for RB-Y1 M v1.3 control UI.

Layout policy
-------------
Always visible:
- global MOTION STOP
- backend mode
- robot state / safety state
- Power / Servo / Stream controls
- current base command
- compact ROS connection summary
- event log

Tabs:
- Base
- Joints
- Scenario
- Diagnostics
"""

from __future__ import annotations

from typing import Dict, Set

from .qt_compat import (
    QAbstractSpinBox,
    QApplication,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFont,
    QFormLayout,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QPlainTextEdit,
    QPushButton,
    QTabWidget,
    QTimer,
    QVBoxLayout,
    QWidget,
    alignment,
    event_type,
    focus_policy,
    qt_key,
)


class MainWindow(QMainWindow):
    ACTION_FORWARD = "forward"
    ACTION_BACKWARD = "backward"
    ACTION_LEFT = "left"
    ACTION_RIGHT = "right"
    ACTION_ROTATE_CCW = "rotate_ccw"
    ACTION_ROTATE_CW = "rotate_cw"

    # UI-level grouping for RB-Y1 body components.
    # Actual ROS joint names and limits are resolved in RosBackend on Day 3.
    JOINT_GROUPS = {
        "right_arm": ("Right Arm", 7),
        "left_arm": ("Left Arm", 7),
        "torso": ("Torso", 6),
        "head": ("Head", 2),
    }

    CARTESIAN_ARMS = {
        "right_arm": "Right Arm",
        "left_arm": "Left Arm",
    }

    def __init__(self, backend) -> None:
        super().__init__()

        self.backend = backend
        self.backend_name = getattr(backend, "backend_name", "ROS2")
        self._pressed_actions: Set[str] = set()
        self._action_buttons: Dict[str, QPushButton] = {}
        self._closing = False

        # Day 2 motion UI state. These caches belong to the UI only;
        # the backend remains the source of truth for actual robot state.
        self._joint_target_cache = {
            key: [0.0] * dof
            for key, (_, dof) in self.JOINT_GROUPS.items()
        }
        self._cartesian_target_cache = {
            key: [0.0] * 6
            for key in self.CARTESIAN_ARMS
        }
        self._latest_motion_state = {}

        self.setWindowTitle(
            f"RB-Y1 M v1.3 Control Center · {self.backend_name}"
        )
        self.setMinimumSize(1180, 780)
        self.resize(1440, 900)
        self.setFocusPolicy(focus_policy("StrongFocus"))

        central = QWidget(self)
        self.setCentralWidget(central)

        root = QVBoxLayout(central)
        root.setContentsMargins(14, 14, 14, 14)
        root.setSpacing(10)

        # --------------------------------------------------------------
        # Fixed top header
        # --------------------------------------------------------------
        root.addWidget(self._build_header())

        # --------------------------------------------------------------
        # Main workspace
        #   left  : fixed operational sidebar
        #   right : task-specific tabs
        # --------------------------------------------------------------
        main_layout = QHBoxLayout()
        main_layout.setSpacing(12)

        self.fixed_status_panel = self._build_fixed_status_panel()
        main_layout.addWidget(self.fixed_status_panel, 0)

        self.tabs = QTabWidget()
        self.tabs.addTab(self._build_base_tab(), "Base")
        self.tabs.addTab(self._build_joint_tab(), "Joints")
        self.tabs.addTab(self._build_scenario_tab(), "Scenario")
        self.tabs.addTab(self._build_diagnostics_tab(), "Diagnostics")
        main_layout.addWidget(self.tabs, 1)

        root.addLayout(main_layout, 1)

        # --------------------------------------------------------------
        # Fixed bottom event log
        # --------------------------------------------------------------
        root.addWidget(self._build_log_panel(), 0)

        self._apply_style()

        # Base commands are refreshed at 25 Hz while a control is held.
        self.command_timer = QTimer(self)
        self.command_timer.setInterval(40)
        self.command_timer.timeout.connect(self._refresh_command)
        self.command_timer.start()

        # UI status refresh can be slower than the command path.
        self.status_timer = QTimer(self)
        self.status_timer.setInterval(200)
        self.status_timer.timeout.connect(self.refresh_backend_status)
        self.status_timer.start()

        self.append_log(
            "info",
            f"UI initialized in {self.backend_name} mode.",
        )
        self.refresh_backend_status()

    # ==================================================================
    # Fixed header
    # ==================================================================
    def _build_header(self) -> QWidget:
        frame = QFrame()
        frame.setObjectName("headerFrame")

        layout = QHBoxLayout(frame)
        layout.setContentsMargins(12, 8, 12, 8)

        title_box = QVBoxLayout()

        title = QLabel("RB-Y1 M v1.3 Control Center")
        font = QFont()
        font.setPointSize(17)
        font.setBold(True)
        title.setFont(font)

        subtitle = QLabel(
            "Robot state and safety controls remain visible at all times"
        )

        title_box.addWidget(title)
        title_box.addWidget(subtitle)

        layout.addLayout(title_box)
        layout.addStretch(1)

        self.backend_badge = QLabel(f"MODE: {self.backend_name}")
        self.backend_badge.setObjectName("backendBadge")
        layout.addWidget(self.backend_badge)

        self.keyboard_enable = QCheckBox("Keyboard base control")
        self.keyboard_enable.setChecked(True)
        layout.addWidget(self.keyboard_enable)

        # Global motion stop stays fixed regardless of selected tab.
        self.stop_button = QPushButton("MOTION STOP")
        self.stop_button.setObjectName("emergencyStop")
        self.stop_button.setMinimumHeight(48)
        self.stop_button.setMinimumWidth(150)
        self.stop_button.clicked.connect(self.emergency_stop)
        layout.addWidget(self.stop_button)

        return frame

    # ==================================================================
    # Fixed left sidebar
    # ==================================================================
    def _build_fixed_status_panel(self) -> QWidget:
        panel = QWidget()
        panel.setObjectName("fixedSidebar")
        panel.setMinimumWidth(285)
        panel.setMaximumWidth(350)

        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(9)

        # Highest priority: robot/safety state.
        layout.addWidget(self._build_robot_state_group())

        # High priority: controls required to prepare or disable the robot.
        layout.addWidget(self._build_robot_service_group())

        # Useful on every tab: what base command is currently being requested.
        layout.addWidget(self._build_current_command_group())

        # Keep only a compact connection summary fixed.
        # Detailed ROS inspection remains in Diagnostics.
        layout.addWidget(self._build_connection_group())

        # Compact warning rather than a large permanent safety text block.
        layout.addWidget(self._build_safety_note_group())

        layout.addStretch(1)
        return panel

    def _build_robot_state_group(self) -> QGroupBox:
        group = QGroupBox("Robot State")
        form = QFormLayout(group)

        self.state_value = QLabel("unknown")
        self.stream_value = QLabel("unknown")
        self.emo_value = QLabel("unknown")
        self.collision_value = QLabel("unknown")

        self.state_value.setObjectName("stateValue")
        self.stream_value.setObjectName("stateValue")
        self.emo_value.setObjectName("stateValue")
        self.collision_value.setObjectName("stateValue")

        form.addRow("Control", self.state_value)
        form.addRow("Stream", self.stream_value)
        form.addRow("EMO", self.emo_value)
        form.addRow("Collision", self.collision_value)

        return group

    def _build_robot_service_group(self) -> QGroupBox:
        group = QGroupBox("Robot Control")
        layout = QGridLayout(group)

        self.prepare_button = QPushButton(
            "Prepare Robot\nPower + Servo"
        )
        self.prepare_button.clicked.connect(
            self.backend.prepare_robot
        )

        self.power_on_button = QPushButton("Power ON")
        self.power_on_button.clicked.connect(
            lambda: self.backend.request_power(True)
        )

        self.power_off_button = QPushButton("Power OFF")
        self.power_off_button.clicked.connect(
            lambda: self.backend.request_power(False)
        )

        self.servo_on_button = QPushButton("Servo ON")
        self.servo_on_button.clicked.connect(
            lambda: self.backend.request_servo(True)
        )

        self.servo_off_button = QPushButton("Servo OFF")
        self.servo_off_button.clicked.connect(
            lambda: self.backend.request_servo(False)
        )

        self.stream_on_button = QPushButton("Stream ON")
        self.stream_on_button.clicked.connect(
            lambda: self.backend.request_stream(True)
        )

        self.stream_off_button = QPushButton("Stream OFF")
        self.stream_off_button.clicked.connect(self._stream_off)

        # Keep labels readable even when the fixed sidebar is narrow.
        self.prepare_button.setMinimumHeight(42)
        for button in (
            self.power_on_button,
            self.power_off_button,
            self.servo_on_button,
            self.servo_off_button,
            self.stream_on_button,
            self.stream_off_button,
        ):
            button.setMinimumHeight(32)

        layout.addWidget(self.prepare_button, 0, 0, 1, 2)
        layout.addWidget(self.power_on_button, 1, 0)
        layout.addWidget(self.power_off_button, 1, 1)
        layout.addWidget(self.servo_on_button, 2, 0)
        layout.addWidget(self.servo_off_button, 2, 1)
        layout.addWidget(self.stream_on_button, 3, 0)
        layout.addWidget(self.stream_off_button, 3, 1)

        return group

    def _build_current_command_group(self) -> QGroupBox:
        group = QGroupBox("Current Base Command")
        form = QFormLayout(group)

        self.vx_value = self._command_label()
        self.vy_value = self._command_label()
        self.wz_value = self._command_label()

        form.addRow("linear.x", self.vx_value)
        form.addRow("linear.y", self.vy_value)
        form.addRow("angular.z", self.wz_value)

        return group

    def _build_connection_group(self) -> QGroupBox:
        group = QGroupBox("Connection")
        form = QFormLayout(group)

        self.backend_value = QLabel(self.backend_name)
        self.namespace_value = QLabel("-")
        self.topic_value = QLabel("-")
        self.subscriber_value = QLabel("0")
        self.service_value = QLabel("unknown")
        self.service_value.setWordWrap(True)

        form.addRow("Backend", self.backend_value)
        form.addRow("Namespace", self.namespace_value)
        form.addRow("cmd_vel", self.topic_value)
        form.addRow("Subscribers", self.subscriber_value)
        form.addRow("Services", self.service_value)

        return group

    @staticmethod
    def _build_safety_note_group() -> QGroupBox:
        group = QGroupBox("Safety")
        layout = QVBoxLayout(group)

        note = QLabel(
            "GUI MOTION STOP sends a stop command, but it does not replace "
            "the robot's physical E-stop."
        )
        note.setWordWrap(True)
        layout.addWidget(note)

        return group

    # ==================================================================
    # Base tab
    # ==================================================================
    def _build_base_tab(self) -> QWidget:
        tab = QWidget()
        layout = QHBoxLayout(tab)
        layout.setSpacing(12)

        left = QVBoxLayout()
        right = QVBoxLayout()

        speed = QGroupBox("Command Settings")
        speed_grid = QGridLayout(speed)

        self.linear_speed = self._speed_spin(
            0.15,
            1.50,
            " m/s",
        )
        self.lateral_speed = self._speed_spin(
            0.15,
            1.50,
            " m/s",
        )
        self.angular_speed = self._speed_spin(
            0.25,
            2.50,
            " rad/s",
        )

        speed_grid.addWidget(
            QLabel("Forward / backward"),
            0,
            0,
        )
        speed_grid.addWidget(self.linear_speed, 0, 1)
        speed_grid.addWidget(QLabel("Lateral"), 1, 0)
        speed_grid.addWidget(self.lateral_speed, 1, 1)
        speed_grid.addWidget(QLabel("Yaw rotation"), 2, 0)
        speed_grid.addWidget(self.angular_speed, 2, 1)

        left.addWidget(speed)

        motion = QGroupBox("Mobile Base Jog Control")
        motion_grid = QGridLayout(motion)
        motion_grid.setHorizontalSpacing(9)
        motion_grid.setVerticalSpacing(9)

        ccw = self._motion_button(
            "↶ Rotate CCW",
            self.ACTION_ROTATE_CCW,
        )
        forward = self._motion_button(
            "↑ Forward",
            self.ACTION_FORWARD,
        )
        cw = self._motion_button(
            "↷ Rotate CW",
            self.ACTION_ROTATE_CW,
        )
        left_button = self._motion_button(
            "← Strafe left",
            self.ACTION_LEFT,
        )
        right_button = self._motion_button(
            "→ Strafe right",
            self.ACTION_RIGHT,
        )
        backward = self._motion_button(
            "↓ Backward",
            self.ACTION_BACKWARD,
        )

        center_stop = QPushButton("STOP · Space")
        center_stop.setObjectName("centerStop")
        center_stop.setMinimumHeight(54)
        center_stop.clicked.connect(self.emergency_stop)

        motion_grid.addWidget(ccw, 0, 0)
        motion_grid.addWidget(forward, 0, 1)
        motion_grid.addWidget(cw, 0, 2)
        motion_grid.addWidget(left_button, 1, 0)
        motion_grid.addWidget(center_stop, 1, 1)
        motion_grid.addWidget(right_button, 1, 2)
        motion_grid.addWidget(backward, 2, 1)

        left.addWidget(motion, 1)

        options = QGroupBox("Base Options")
        option_layout = QVBoxLayout(options)

        self.require_stream = QCheckBox(
            "Allow non-zero command only when Stream is ON"
        )
        self.require_stream.setChecked(True)

        self.stop_on_focus_loss = QCheckBox(
            "STOP when window loses focus"
        )
        self.stop_on_focus_loss.setChecked(True)

        self.invert_lateral = QCheckBox(
            "Invert lateral direction"
        )

        self.arrow_mode = QComboBox()
        self.arrow_mode.addItem(
            "←/→ = lateral strafe",
            "strafe",
        )
        self.arrow_mode.addItem(
            "←/→ = yaw rotation",
            "rotate",
        )

        option_layout.addWidget(self.require_stream)
        option_layout.addWidget(self.stop_on_focus_loss)
        option_layout.addWidget(self.invert_lateral)
        option_layout.addWidget(QLabel("Arrow-key mode"))
        option_layout.addWidget(self.arrow_mode)

        right.addWidget(options)

        usage = QGroupBox("Operating Note")
        usage_layout = QVBoxLayout(usage)

        usage_text = QLabel(
            "Hold a jog button to command motion. Releasing the button "
            "returns the target base command to zero. "
            "Space or Esc also triggers MOTION STOP."
        )
        usage_text.setWordWrap(True)
        usage_layout.addWidget(usage_text)

        right.addWidget(usage)
        right.addStretch(1)

        layout.addLayout(left, 3)
        layout.addLayout(right, 2)

        return tab

    @staticmethod
    def _speed_spin(
        value: float,
        maximum: float,
        suffix: str,
    ) -> QDoubleSpinBox:
        spin = QDoubleSpinBox()
        spin.setDecimals(2)
        spin.setRange(0.01, maximum)
        spin.setSingleStep(0.01)
        spin.setValue(value)
        spin.setSuffix(suffix)
        spin.setKeyboardTracking(False)
        return spin

    def _motion_button(
        self,
        text: str,
        action: str,
    ) -> QPushButton:
        button = QPushButton(text)
        button.setMinimumHeight(54)
        button.setAutoRepeat(False)

        button.pressed.connect(
            lambda selected=action: self._press_action(selected)
        )
        button.released.connect(
            lambda selected=action: self._release_action(selected)
        )

        self._action_buttons[action] = button
        return button

    @staticmethod
    def _command_label() -> QLabel:
        label = QLabel("0.00")
        font = QFont("Monospace")
        font.setPointSize(12)
        font.setBold(True)
        label.setFont(font)
        label.setAlignment(alignment("AlignRight"))
        return label

    # ==================================================================
    # Placeholder tabs for later implementation
    # ==================================================================
    def _placeholder(
        self,
        title: str,
        text: str,
    ) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)

        group = QGroupBox(title)
        group_layout = QVBoxLayout(group)

        label = QLabel(text)
        label.setWordWrap(True)

        group_layout.addWidget(label)
        group_layout.addStretch(1)

        layout.addWidget(group)
        return tab

    # ==================================================================
    # Day 2: Joint / Cartesian motion tab
    # ==================================================================
    def _build_joint_tab(self) -> QWidget:
        tab = QWidget()
        root = QVBoxLayout(tab)
        root.setSpacing(8)

        # --------------------------------------------------------------
        # Joint / Cartesian motion panels
        # --------------------------------------------------------------
        motion_layout = QHBoxLayout()
        motion_layout.setSpacing(12)

        joint_panel = self._build_joint_space_panel()
        cartesian_panel = self._build_cartesian_space_panel()

        motion_layout.addWidget(joint_panel, 1)
        motion_layout.addWidget(cartesian_panel, 1)

        root.addLayout(motion_layout, 1)

        # --------------------------------------------------------------
        # Shared motion command area
        # --------------------------------------------------------------
        action_group = QGroupBox("Motion Command")
        action_layout = QGridLayout(action_group)
        action_layout.setHorizontalSpacing(10)
        action_layout.setVerticalSpacing(6)

        self.motion_type_selector = QComboBox()
        self.motion_type_selector.addItem("Joint Space", "joint")
        self.motion_type_selector.addItem("Cartesian Space", "cartesian")

        self.copy_current_button = QPushButton("Copy Current → Target")
        self.move_target_button = QPushButton("MOVE TARGET")
        self.cancel_motion_button = QPushButton("CANCEL")

        self.copy_current_button.clicked.connect(
            self._copy_selected_current_to_target
        )
        self.move_target_button.clicked.connect(
            self._move_selected_target
        )
        self.cancel_motion_button.clicked.connect(
            self._cancel_arm_motion
        )

        action_layout.addWidget(QLabel("Motion Type"), 0, 0)
        action_layout.addWidget(self.motion_type_selector, 0, 1)

        action_layout.addWidget(
            self.copy_current_button,
            1,
            0,
            1,
            2,
        )
        action_layout.addWidget(self.move_target_button, 2, 0)
        action_layout.addWidget(self.cancel_motion_button, 2, 1)

        root.addWidget(action_group)

        return tab

    def _build_joint_space_panel(self) -> QGroupBox:
        panel = QGroupBox("Joint Space Motion")
        root = QVBoxLayout(panel)
        root.setSpacing(9)

        # --------------------------------------------------------------
        # Component selector
        # --------------------------------------------------------------
        selector_group = QGroupBox("Joint Group")
        selector_layout = QFormLayout(selector_group)

        self.joint_group_selector = QComboBox()
        for key, (label, _) in self.JOINT_GROUPS.items():
            self.joint_group_selector.addItem(label, key)

        self.joint_group_selector.currentIndexChanged.connect(
            self._on_joint_group_changed
        )
        selector_layout.addRow("Component", self.joint_group_selector)
        root.addWidget(selector_group)

        # --------------------------------------------------------------
        # Current / target / jog table
        # --------------------------------------------------------------
        state_group = QGroupBox("Joint Position")
        grid = QGridLayout(state_group)
        grid.setHorizontalSpacing(10)
        grid.setVerticalSpacing(6)
        grid.setColumnStretch(0, 0)
        grid.setColumnStretch(1, 1)
        grid.setColumnStretch(2, 1)
        grid.setColumnStretch(3, 0)
        grid.setColumnStretch(4, 0)

        joint_header = QLabel("Joint")
        current_header = QLabel("Current [deg]")
        target_header = QLabel("Target [deg]")
        jog_header = QLabel("Jog")

        current_header.setAlignment(alignment("AlignCenter"))
        target_header.setAlignment(alignment("AlignCenter"))
        jog_header.setAlignment(alignment("AlignCenter"))

        grid.addWidget(joint_header, 0, 0)
        grid.addWidget(current_header, 0, 1)
        grid.addWidget(target_header, 0, 2)
        grid.addWidget(jog_header, 0, 3, 1, 2)

        self.joint_name_labels = []
        self.joint_current_labels = []
        self.joint_target_spins = []
        self.joint_minus_buttons = []
        self.joint_plus_buttons = []
        self.joint_row_widgets = []

        # Maximum visible group is a 7-DOF arm.
        for index in range(7):
            row = index + 1

            name = QLabel(f"J{index + 1}")
            current = QLabel("--")
            current.setAlignment(alignment("AlignCenter"))
            current.setMinimumWidth(82)

            target = QDoubleSpinBox()
            target.setMinimumWidth(105)
            target.setDecimals(2)
            target.setRange(-360.0, 360.0)
            target.setSingleStep(1.0)
            target.setSuffix("°")
            target.setKeyboardTracking(False)
            target.valueChanged.connect(
                lambda value, i=index: self._on_joint_target_changed(i, value)
            )

            minus = QPushButton("−")
            plus = QPushButton("+")
            minus.setFixedWidth(38)
            plus.setFixedWidth(38)

            minus.clicked.connect(
                lambda checked=False, i=index: self._joint_jog(i, -1)
            )
            plus.clicked.connect(
                lambda checked=False, i=index: self._joint_jog(i, +1)
            )

            grid.addWidget(name, row, 0)
            grid.addWidget(current, row, 1)
            grid.addWidget(target, row, 2)
            grid.addWidget(minus, row, 3)
            grid.addWidget(plus, row, 4)

            self.joint_name_labels.append(name)
            self.joint_current_labels.append(current)
            self.joint_target_spins.append(target)
            self.joint_minus_buttons.append(minus)
            self.joint_plus_buttons.append(plus)
            self.joint_row_widgets.append(
                (name, current, target, minus, plus)
            )

        root.addWidget(state_group, 1)

        # --------------------------------------------------------------
        # Joint-space settings
        # --------------------------------------------------------------
        settings = QGroupBox("Joint Motion Settings")
        settings_layout = QGridLayout(settings)

        self.joint_jog_step = QDoubleSpinBox()
        self.joint_jog_step.setDecimals(1)
        self.joint_jog_step.setRange(0.1, 30.0)
        self.joint_jog_step.setSingleStep(0.5)
        self.joint_jog_step.setValue(2.0)
        self.joint_jog_step.setSuffix("° / click")

        self.joint_minimum_time = QDoubleSpinBox()
        self.joint_minimum_time.setDecimals(1)
        self.joint_minimum_time.setRange(0.1, 30.0)
        self.joint_minimum_time.setSingleStep(0.5)
        self.joint_minimum_time.setValue(3.0)
        self.joint_minimum_time.setSuffix(" s")

        settings_layout.addWidget(QLabel("Jog step"), 0, 0)
        settings_layout.addWidget(self.joint_jog_step, 0, 1)
        settings_layout.addWidget(QLabel("Minimum time"), 1, 0)
        settings_layout.addWidget(self.joint_minimum_time, 1, 1)

        root.addWidget(settings)

        self._on_joint_group_changed()
        return panel

    def _build_cartesian_space_panel(self) -> QGroupBox:
        panel = QGroupBox("Cartesian Space Motion")
        root = QVBoxLayout(panel)
        root.setSpacing(9)

        # --------------------------------------------------------------
        # End-effector selector
        # Reference frame is intentionally fixed to "base" in Day 3.
        # --------------------------------------------------------------
        selector_group = QGroupBox("End Effector")
        selector_layout = QFormLayout(selector_group)

        self.cartesian_arm_selector = QComboBox()
        for key, label in self.CARTESIAN_ARMS.items():
            self.cartesian_arm_selector.addItem(label, key)

        self.cartesian_arm_selector.currentIndexChanged.connect(
            self._on_cartesian_arm_changed
        )

        selector_layout.addRow("Arm", self.cartesian_arm_selector)
        root.addWidget(selector_group)

        # --------------------------------------------------------------
        # Cartesian current / target / jog table
        # Same interaction pattern as Joint Space Motion.
        # --------------------------------------------------------------
        motion_group = QGroupBox("TCP Motion")
        grid = QGridLayout(motion_group)
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(6)

        grid.setColumnStretch(0, 0)
        grid.setColumnStretch(1, 1)
        grid.setColumnStretch(2, 2)
        grid.setColumnStretch(3, 0)
        grid.setColumnStretch(4, 0)

        axis_header = QLabel("Axis")
        current_header = QLabel("Current")
        target_header = QLabel("Target")
        jog_header = QLabel("Jog")

        current_header.setAlignment(alignment("AlignCenter"))
        target_header.setAlignment(alignment("AlignCenter"))
        jog_header.setAlignment(alignment("AlignCenter"))

        grid.addWidget(axis_header, 0, 0)
        grid.addWidget(current_header, 0, 1)
        grid.addWidget(target_header, 0, 2)
        grid.addWidget(jog_header, 0, 3, 1, 2)

        axis_specs = (
            ("X", " m", -2.0, 2.0, 3),
            ("Y", " m", -2.0, 2.0, 3),
            ("Z", " m", -2.0, 2.0, 3),
            ("Roll", "°", -180.0, 180.0, 1),
            ("Pitch", "°", -180.0, 180.0, 1),
            ("Yaw", "°", -180.0, 180.0, 1),
        )

        self.cartesian_current_labels = []
        self.cartesian_target_spins = []
        self.cartesian_minus_buttons = []
        self.cartesian_plus_buttons = []

        for index, (axis, suffix, minimum, maximum, decimals) in enumerate(
            axis_specs
        ):
            row = index + 1

            axis_label = QLabel(axis)

            current = QLabel("--")
            current.setAlignment(alignment("AlignCenter"))
            current.setMinimumWidth(90)

            target = QDoubleSpinBox()
            target.setMinimumWidth(150)
            target.setDecimals(decimals)
            target.setRange(minimum, maximum)
            target.setSingleStep(0.01 if index < 3 else 1.0)
            target.setSuffix(suffix)
            target.setKeyboardTracking(False)
            target.valueChanged.connect(
                lambda value, i=index: self._on_cartesian_target_changed(i, value)
            )

            minus = QPushButton("−")
            plus = QPushButton("+")
            minus.setFixedSize(38, 28)
            plus.setFixedSize(38, 28)

            minus.clicked.connect(
                lambda checked=False, i=index: self._cartesian_jog(i, -1)
            )
            plus.clicked.connect(
                lambda checked=False, i=index: self._cartesian_jog(i, +1)
            )

            grid.addWidget(axis_label, row, 0)
            grid.addWidget(current, row, 1)
            grid.addWidget(target, row, 2)
            grid.addWidget(minus, row, 3)
            grid.addWidget(plus, row, 4)

            self.cartesian_current_labels.append(current)
            self.cartesian_target_spins.append(target)
            self.cartesian_minus_buttons.append(minus)
            self.cartesian_plus_buttons.append(plus)

        root.addWidget(motion_group, 1)

        # --------------------------------------------------------------
        # Cartesian motion settings
        # --------------------------------------------------------------
        settings = QGroupBox("Cartesian Motion Settings")
        settings_layout = QGridLayout(settings)
        settings_layout.setHorizontalSpacing(10)
        settings_layout.setVerticalSpacing(6)

        self.cartesian_linear_step = QDoubleSpinBox()
        self.cartesian_linear_step.setDecimals(3)
        self.cartesian_linear_step.setRange(0.001, 0.200)
        self.cartesian_linear_step.setSingleStep(0.005)
        self.cartesian_linear_step.setValue(0.010)
        self.cartesian_linear_step.setSuffix(" m / click")

        self.cartesian_angular_step = QDoubleSpinBox()
        self.cartesian_angular_step.setDecimals(1)
        self.cartesian_angular_step.setRange(0.5, 30.0)
        self.cartesian_angular_step.setSingleStep(0.5)
        self.cartesian_angular_step.setValue(5.0)
        self.cartesian_angular_step.setSuffix("° / click")

        self.cartesian_minimum_time = QDoubleSpinBox()
        self.cartesian_minimum_time.setDecimals(1)
        self.cartesian_minimum_time.setRange(0.1, 30.0)
        self.cartesian_minimum_time.setSingleStep(0.5)
        self.cartesian_minimum_time.setValue(3.0)
        self.cartesian_minimum_time.setSuffix(" s")

        settings_layout.addWidget(QLabel("Linear jog"), 0, 0)
        settings_layout.addWidget(self.cartesian_linear_step, 0, 1)
        settings_layout.addWidget(QLabel("Angular jog"), 1, 0)
        settings_layout.addWidget(self.cartesian_angular_step, 1, 1)
        settings_layout.addWidget(QLabel("Minimum time"), 2, 0)
        settings_layout.addWidget(self.cartesian_minimum_time, 2, 1)

        root.addWidget(settings)

        self._on_cartesian_arm_changed()
        return panel

    # ------------------------------------------------------------------
    # Joint-space UI callbacks
    # ------------------------------------------------------------------
    def _selected_joint_group(self) -> str:
        return str(self.joint_group_selector.currentData())

    def _on_joint_group_changed(self, *args) -> None:
        del args
        if not hasattr(self, "joint_group_selector"):
            return

        group = self._selected_joint_group()
        label, dof = self.JOINT_GROUPS[group]
        cached_targets = self._joint_target_cache[group]

        for index, row_widgets in enumerate(self.joint_row_widgets):
            visible = index < dof
            for widget in row_widgets:
                widget.setVisible(visible)

            if not visible:
                continue

            self.joint_name_labels[index].setText(f"J{index + 1}")
            self.joint_target_spins[index].blockSignals(True)
            self.joint_target_spins[index].setValue(cached_targets[index])
            self.joint_target_spins[index].blockSignals(False)

        if hasattr(self, "log_view"):
            self.append_log(
                "info",
                f"Joint group selected: {label}.",
            )

    def _on_joint_target_changed(
        self,
        index: int,
        value: float,
    ) -> None:
        group = self._selected_joint_group()
        _, dof = self.JOINT_GROUPS[group]
        if index < dof:
            self._joint_target_cache[group][index] = float(value)

    def _joint_jog(
        self,
        index: int,
        direction: int,
    ) -> None:
        group = self._selected_joint_group()
        _, dof = self.JOINT_GROUPS[group]
        if index >= dof:
            return

        step_deg = float(self.joint_jog_step.value()) * float(direction)

        if not hasattr(self.backend, "jog_joint"):
            self.append_log(
                "warning",
                "Joint jog is not available in this backend yet.",
            )
            return

        self.backend.jog_joint(
            group,
            index,
            step_deg,
        )

    def _move_joint_target(self) -> None:
        group = self._selected_joint_group()
        _, dof = self.JOINT_GROUPS[group]

        targets_deg = [
            float(self.joint_target_spins[i].value())
            for i in range(dof)
        ]
        self._joint_target_cache[group] = list(targets_deg)

        if not hasattr(self.backend, "move_joint_group"):
            self.append_log(
                "warning",
                "Joint target motion is not available in this backend yet.",
            )
            return

        self.backend.move_joint_group(
            group,
            targets_deg,
            float(self.joint_minimum_time.value()),
        )

    def _copy_current_joint_to_target(self) -> None:
        group = self._selected_joint_group()
        state = self._latest_motion_state.get("joint_groups", {})
        current = state.get(group)

        if current is None:
            self.append_log(
                "warning",
                "No current joint state is available to copy.",
            )
            return

        _, dof = self.JOINT_GROUPS[group]
        values = [float(v) for v in current[:dof]]
        self._joint_target_cache[group] = values

        for index, value in enumerate(values):
            self.joint_target_spins[index].blockSignals(True)
            self.joint_target_spins[index].setValue(value)
            self.joint_target_spins[index].blockSignals(False)

    # ------------------------------------------------------------------
    # Cartesian-space UI callbacks
    # ------------------------------------------------------------------
    def _selected_cartesian_arm(self) -> str:
        return str(self.cartesian_arm_selector.currentData())

    def _on_cartesian_arm_changed(self, *args) -> None:
        del args
        if not hasattr(self, "cartesian_arm_selector"):
            return

        arm = self._selected_cartesian_arm()
        cached = self._cartesian_target_cache[arm]

        for index, value in enumerate(cached):
            self.cartesian_target_spins[index].blockSignals(True)
            self.cartesian_target_spins[index].setValue(value)
            self.cartesian_target_spins[index].blockSignals(False)

        if hasattr(self, "log_view"):
            self.append_log(
                "info",
                f"Cartesian arm selected: {self.CARTESIAN_ARMS[arm]}.",
            )

    def _on_cartesian_target_changed(
        self,
        index: int,
        value: float,
    ) -> None:
        arm = self._selected_cartesian_arm()
        self._cartesian_target_cache[arm][index] = float(value)

    def _cartesian_jog(
        self,
        axis_index: int,
        direction: int,
    ) -> None:
        arm = self._selected_cartesian_arm()

        if axis_index < 3:
            delta = float(self.cartesian_linear_step.value())
        else:
            delta = float(self.cartesian_angular_step.value())

        delta *= float(direction)

        if not hasattr(self.backend, "jog_cartesian"):
            self.append_log(
                "warning",
                "Cartesian jog is not available in this backend yet.",
            )
            return

        self.backend.jog_cartesian(
            arm,
            axis_index,
            delta,
            reference_frame="base",
        )

    def _move_cartesian_target(self) -> None:
        arm = self._selected_cartesian_arm()

        target = [
            float(spin.value())
            for spin in self.cartesian_target_spins
        ]
        self._cartesian_target_cache[arm] = list(target)

        if not hasattr(self.backend, "move_cartesian"):
            self.append_log(
                "warning",
                "Cartesian target motion is not available in this backend yet.",
            )
            return

        self.backend.move_cartesian(
            arm,
            target,
            float(self.cartesian_minimum_time.value()),
            reference_frame="base",
        )

    def _copy_current_cartesian_to_target(self) -> None:
        arm = self._selected_cartesian_arm()
        state = self._latest_motion_state.get("cartesian", {})
        current = state.get(arm)

        if current is None:
            self.append_log(
                "warning",
                "No current Cartesian state is available to copy.",
            )
            return

        values = [float(v) for v in current[:6]]
        self._cartesian_target_cache[arm] = values

        for index, value in enumerate(values):
            self.cartesian_target_spins[index].blockSignals(True)
            self.cartesian_target_spins[index].setValue(value)
            self.cartesian_target_spins[index].blockSignals(False)

    def _copy_selected_current_to_target(self) -> None:
        mode = str(self.motion_type_selector.currentData())

        if mode == "joint":
            self._copy_current_joint_to_target()
        elif mode == "cartesian":
            self._copy_current_cartesian_to_target()

    def _move_selected_target(self) -> None:
        mode = str(self.motion_type_selector.currentData())

        if mode == "joint":
            self._move_joint_target()
        elif mode == "cartesian":
            self._move_cartesian_target()

    def _cancel_arm_motion(self) -> None:
        if hasattr(self.backend, "cancel_motion"):
            self.backend.cancel_motion()
        else:
            self.append_log(
                "warning",
                "Arm motion cancel is not available in this backend yet.",
            )

    def _refresh_motion_state(self) -> None:
        if not hasattr(self.backend, "get_motion_state"):
            return

        state = self.backend.get_motion_state()
        if not isinstance(state, dict):
            return

        self._latest_motion_state = state

        # Joint state: only the selected group is displayed.
        group = self._selected_joint_group()
        _, dof = self.JOINT_GROUPS[group]
        joint_values = state.get("joint_groups", {}).get(group)

        if joint_values is not None:
            for index in range(dof):
                if index < len(joint_values):
                    self.joint_current_labels[index].setText(
                        f"{float(joint_values[index]):+.2f}"
                    )
                else:
                    self.joint_current_labels[index].setText("--")

        # Cartesian state: only the selected arm is displayed.
        arm = self._selected_cartesian_arm()
        cartesian = state.get("cartesian", {}).get(arm)

        if cartesian is not None:
            for index in range(6):
                if index >= len(cartesian):
                    self.cartesian_current_labels[index].setText("--")
                    continue

                value = float(cartesian[index])
                if index < 3:
                    self.cartesian_current_labels[index].setText(
                        f"{value:+.4f} m"
                    )
                else:
                    self.cartesian_current_labels[index].setText(
                        f"{value:+.2f}°"
                    )

    def _build_scenario_tab(self) -> QWidget:
        return self._placeholder(
            "Scenario Manager · next implementation",
            "Scenario execution will use explicit steps such as "
            "READY → BASE MOVE → STOP → JOINT POSE → RESULT, "
            "with Cancel / Timeout / failure logging.",
        )

    def _build_diagnostics_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)

        group = QGroupBox("Diagnostics")
        group_layout = QVBoxLayout(group)

        description = QLabel(
            "Detailed ROS information belongs here rather than in the "
            "always-visible operator area."
        )
        description.setWordWrap(True)

        run_button = QPushButton("Run Diagnostics")
        run_button.clicked.connect(self.run_diagnostics)

        self.diagnostic_summary = QPlainTextEdit()
        self.diagnostic_summary.setReadOnly(True)

        group_layout.addWidget(description)
        group_layout.addWidget(run_button)
        group_layout.addWidget(self.diagnostic_summary)

        layout.addWidget(group)
        return tab

    def run_diagnostics(self) -> None:
        if hasattr(self.backend, "run_diagnostics"):
            data = self.backend.run_diagnostics()
            lines = [
                f"{key}: {value}"
                for key, value in data.items()
            ]
        else:
            snapshot = self.backend.snapshot()
            lines = [
                f"backend: {self.backend_name}",
                f"namespace: {snapshot.namespace}",
                f"cmd_vel_topic: {snapshot.cmd_vel_topic}",
                (
                    "cmd_vel_subscribers: "
                    f"{snapshot.cmd_vel_subscribers}"
                ),
                f"control_state: {snapshot.control_state}",
                f"stream_enabled: {snapshot.stream_enabled}",
                f"emo_active: {snapshot.emo_active}",
                (
                    "collision_active: "
                    f"{snapshot.collision_active}"
                ),
                (
                    "services_enabled: "
                    f"{snapshot.services_enabled}"
                ),
                f"service_ready: {snapshot.service_ready}",
            ]

        self.diagnostic_summary.setPlainText(
            "\n".join(lines)
        )
        self.append_log(
            "info",
            "Diagnostics refreshed.",
        )

    # ==================================================================
    # Fixed bottom log
    # ==================================================================
    def _build_log_panel(self) -> QGroupBox:
        group = QGroupBox("Event Log")
        layout = QVBoxLayout(group)

        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumBlockCount(700)
        self.log_view.setMinimumHeight(85)
        self.log_view.setMaximumHeight(135)

        layout.addWidget(self.log_view)
        return group

    def append_log(
        self,
        level: str,
        message: str,
    ) -> None:
        prefix = {
            "error": "[ERROR] ",
            "warning": "[WARN] ",
        }.get(level, "")

        self.log_view.appendPlainText(
            prefix + message
        )

        scrollbar = self.log_view.verticalScrollBar()
        scrollbar.setValue(
            scrollbar.maximum()
        )

    # ==================================================================
    # Base command handling
    # ==================================================================
    def _press_action(
        self,
        action: str,
    ) -> None:
        self._pressed_actions.add(action)
        self._refresh_command()

    def _release_action(
        self,
        action: str,
    ) -> None:
        self._pressed_actions.discard(action)
        self._refresh_command()

    def _stream_off(self) -> None:
        # Stop first, then disable stream.
        self.emergency_stop()
        self.backend.request_stream(False)

    def _stop_base_only(self) -> None:
        """Stop only mobile-base motion."""

        self._pressed_actions.clear()

        self.backend.stop(
            publish_immediately=True
        )

        self._update_command_labels(
            0.0,
            0.0,
            0.0,
        )

    def emergency_stop(self) -> None:
        # Stop mobile base first.
        self._stop_base_only()

        # Cancel active joint / Cartesian motion.
        if hasattr(self.backend, "cancel_motion"):
            self.backend.cancel_motion()

    def _refresh_command(self) -> None:
        if self._closing:
            return

        vx, vy, wz = self._calculate_command()
        snapshot = self.backend.snapshot()

        if (
            self.require_stream.isChecked()
            and snapshot.services_enabled
            and snapshot.stream_enabled is not True
        ):
            vx = 0.0
            vy = 0.0
            wz = 0.0

        self.backend.set_velocity(
            vx,
            vy,
            wz,
        )
        self._update_command_labels(
            vx,
            vy,
            wz,
        )

    def _calculate_command(self):
        linear = float(
            self.linear_speed.value()
        )
        lateral = float(
            self.lateral_speed.value()
        )
        angular = float(
            self.angular_speed.value()
        )

        vx = linear * (
            int(
                self.ACTION_FORWARD
                in self._pressed_actions
            )
            - int(
                self.ACTION_BACKWARD
                in self._pressed_actions
            )
        )

        lateral_direction = (
            int(
                self.ACTION_LEFT
                in self._pressed_actions
            )
            - int(
                self.ACTION_RIGHT
                in self._pressed_actions
            )
        )

        if self.invert_lateral.isChecked():
            lateral_direction *= -1

        vy = lateral * lateral_direction

        wz = angular * (
            int(
                self.ACTION_ROTATE_CCW
                in self._pressed_actions
            )
            - int(
                self.ACTION_ROTATE_CW
                in self._pressed_actions
            )
        )

        return vx, vy, wz

    def _update_command_labels(
        self,
        vx: float,
        vy: float,
        wz: float,
    ) -> None:
        self.vx_value.setText(
            f"{vx:+.2f} m/s"
        )
        self.vy_value.setText(
            f"{vy:+.2f} m/s"
        )
        self.wz_value.setText(
            f"{wz:+.2f} rad/s"
        )

    # ==================================================================
    # Backend state rendering
    # ==================================================================
    def refresh_backend_status(self) -> None:
        snapshot = self.backend.snapshot()

        self.backend_value.setText(
            self.backend_name
        )
        self.namespace_value.setText(
            snapshot.namespace
        )
        self.topic_value.setText(
            snapshot.cmd_vel_topic
        )
        self.subscriber_value.setText(
            str(snapshot.cmd_vel_subscribers)
        )

        self.state_value.setText(
            "unknown"
            if snapshot.control_state is None
            else str(snapshot.control_state)
        )

        self.stream_value.setText(
            "ON"
            if snapshot.stream_enabled is True
            else "OFF"
            if snapshot.stream_enabled is False
            else "unknown"
        )

        self.emo_value.setText(
            "ACTIVE"
            if snapshot.emo_active is True
            else "RELEASED"
            if snapshot.emo_active is False
            else "unknown"
        )

        self.collision_value.setText(
            "ACTIVE"
            if snapshot.collision_active is True
            else "CLEAR"
            if snapshot.collision_active is False
            else "unknown"
        )

        if snapshot.services_enabled:
            ready = snapshot.service_ready
            self.service_value.setText(
                f'power={"ready" if ready.get("power") else "wait"}\n'
                f'servo={"ready" if ready.get("servo") else "wait"}\n'
                f'stream={"ready" if ready.get("stream") else "wait"}'
            )
        else:
            self.service_value.setText(
                "cmd_vel-only mode"
            )

        service_widgets = (
            self.prepare_button,
            self.power_on_button,
            self.power_off_button,
            self.servo_on_button,
            self.servo_off_button,
            self.stream_on_button,
            self.stream_off_button,
        )

        for widget in service_widgets:
            widget.setEnabled(
                snapshot.services_enabled
            )

        # Day 2 motion state, when the backend provides it.
        self._refresh_motion_state()

        for level, message in self.backend.drain_events():
            self.append_log(
                level,
                message,
            )

    # ==================================================================
    # Keyboard / focus safety
    # ==================================================================
    def eventFilter(
        self,
        watched,
        event,
    ):  # noqa: N802
        event_kind = event.type()

        if event_kind == event_type(
            "WindowDeactivate"
        ):
            if (watched is self and self.stop_on_focus_loss.isChecked()):
                self._stop_base_only()
            return False
        if (
            not self.keyboard_enable.isChecked()
            or not self.isActiveWindow()
        ):
            return False

        focused = QApplication.focusWidget()

        if isinstance(
            focused,
            (
                QAbstractSpinBox,
                QLineEdit,
                QComboBox,
            ),
        ):
            return False

        if event_kind not in (
            event_type("KeyPress"),
            event_type("KeyRelease"),
        ):
            return False

        if event.isAutoRepeat():
            return True

        key = event.key()

        if key in (
            qt_key("Key_Space"),
            qt_key("Key_Escape"),
        ):
            if event_kind == event_type(
                "KeyPress"
            ):
                self.emergency_stop()
            return True

        action = self._action_for_key(key)

        if action is None:
            return False

        if event_kind == event_type(
            "KeyPress"
        ):
            self._press_action(action)
        else:
            self._release_action(action)

        return True

    def _action_for_key(
        self,
        key: int,
    ):
        mapping = {
            qt_key("Key_Up"): self.ACTION_FORWARD,
            qt_key("Key_W"): self.ACTION_FORWARD,
            qt_key("Key_Down"): self.ACTION_BACKWARD,
            qt_key("Key_S"): self.ACTION_BACKWARD,
            qt_key("Key_A"): self.ACTION_LEFT,
            qt_key("Key_D"): self.ACTION_RIGHT,
            qt_key("Key_Q"): self.ACTION_ROTATE_CCW,
            qt_key("Key_E"): self.ACTION_ROTATE_CW,
        }

        if self.arrow_mode.currentData() == "rotate":
            mapping[
                qt_key("Key_Left")
            ] = self.ACTION_ROTATE_CCW
            mapping[
                qt_key("Key_Right")
            ] = self.ACTION_ROTATE_CW
        else:
            mapping[
                qt_key("Key_Left")
            ] = self.ACTION_LEFT
            mapping[
                qt_key("Key_Right")
            ] = self.ACTION_RIGHT

        return mapping.get(key)

    # ==================================================================
    # Shutdown
    # ==================================================================
    def closeEvent(self, event):  # noqa: N802
        self._closing = True
        self.command_timer.stop()
        self.status_timer.stop()
        self._pressed_actions.clear()

        try:
            self.backend.shutdown_safely(
                turn_stream_off=True
            )
        finally:
            event.accept()

    # ==================================================================
    # Style
    # ==================================================================
    def _apply_style(self) -> None:
        self.setStyleSheet(
            """
            QMainWindow {
                background: #20242b;
                color: #e8edf2;
            }

            QWidget {
                color: #e8edf2;
                font-size: 13px;
            }

            QGroupBox {
                border: 1px solid #4b5563;
                border-radius: 7px;
                margin-top: 12px;
                padding-top: 8px;
                font-weight: 600;
            }

            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
            }

            QPushButton {
                background: #343b46;
                border: 1px solid #657184;
                border-radius: 6px;
                padding: 8px;
                font-weight: 600;
            }

            QPushButton:hover {
                background: #414b59;
            }

            QPushButton:pressed {
                background: #2c6e9b;
            }

            QPushButton:disabled {
                color: #7d8794;
                background: #2a2f36;
            }

            QPushButton#centerStop {
                background: #7f4a24;
            }

            QPushButton#emergencyStop {
                background: #9b2c2c;
                font-size: 14px;
            }

            QDoubleSpinBox,
            QComboBox,
            QPlainTextEdit {
                background: #171a1f;
                border: 1px solid #4b5563;
                border-radius: 4px;
                padding: 5px;
            }

            QFrame#headerFrame {
                background: #292f38;
                border-radius: 8px;
            }

            QTabWidget::pane {
                border: 1px solid #4b5563;
                border-radius: 6px;
            }

            QTabBar::tab {
                background: #2a2f36;
                padding: 9px 16px;
                margin-right: 2px;
            }

            QTabBar::tab:selected {
                background: #414b59;
            }

            QLabel#backendBadge {
                background: #171a1f;
                border: 1px solid #657184;
                border-radius: 5px;
                padding: 7px 10px;
                font-weight: 700;
            }

            QLabel#stateValue {
                font-weight: 700;
            }

            QLabel#secondaryNote {
                color: #aeb8c4;
                font-size: 12px;
            }
            """
        )

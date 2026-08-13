"""Day 1 Qt shell for RB-Y1 M v1.3 control UI."""



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

    def __init__(self, backend) -> None:
        super().__init__()

        self.backend = backend
        self.backend_name = getattr(backend, "backend_name", "ROS2")
        self._pressed_actions: Set[str] = set()
        self._action_buttons: Dict[str, QPushButton] = {}
        self._closing = False

        self.setWindowTitle(
            f"RB-Y1 M v1.3 Control Center · {self.backend_name}"
        )
        self.setMinimumSize(1050, 740)
        self.resize(1240, 860)
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

    def _build_joint_tab(self) -> QWidget:
        return self._placeholder(
            "Joint Control · next implementation",
            "Day 1 shell is ready. Next: joint_states subscription, current "
            "angles, target angles, RB-Y1 joint Action, Cancel, and "
            "joint-limit checks.",
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
        self.log_view.setMinimumHeight(105)
        self.log_view.setMaximumHeight(165)

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

    def emergency_stop(self) -> None:
        self._pressed_actions.clear()
        self.backend.stop(
            publish_immediately=True
        )
        self._update_command_labels(
            0.0,
            0.0,
            0.0,
        )

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
        del watched
        event_kind = event.type()

        if event_kind == event_type(
            "WindowDeactivate"
        ):
            if self.stop_on_focus_loss.isChecked():
                self.emergency_stop()
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
            """
        )

"""Qt main window for RB-Y1 mobile-base teleoperation."""

from __future__ import annotations

from typing import Dict, Set

from .qt_compat import (
    QAbstractSpinBox,
    QApplication,
    QCheckBox,
    QCloseEvent,
    QComboBox,
    QDoubleSpinBox,
    QEvent,
    QFont,
    QFormLayout,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QTimer,
    QVBoxLayout,
    QWidget,
    alignment,
    event_type,
    focus_policy,
    qt_key,
)
from .ros_backend import BackendSnapshot, Rby1ControlNode


class MainWindow(QMainWindow):
    """Reusable UI that maps held buttons/keys to cmd_vel."""

    ACTION_FORWARD = 'forward'
    ACTION_BACKWARD = 'backward'
    ACTION_LEFT = 'left'
    ACTION_RIGHT = 'right'
    ACTION_ROTATE_CCW = 'rotate_ccw'
    ACTION_ROTATE_CW = 'rotate_cw'

    def __init__(self, node: Rby1ControlNode) -> None:
        super().__init__()
        self.node = node
        self._pressed_actions: Set[str] = set()
        self._action_buttons: Dict[str, QPushButton] = {}
        self._last_status_text = ''
        self._closing = False

        self.setWindowTitle('RB-Y1 Universal Control UI')
        # Keep the window usable at the size shown in the user's screenshot,
        # while opening a little larger when screen space is available.
        self.setMinimumSize(980, 720)
        self.resize(1160, 840)
        self.setFocusPolicy(focus_policy('StrongFocus'))

        central = QWidget(self)
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(14, 14, 14, 14)
        root.setSpacing(10)

        root.addWidget(self._build_header())
        body = QHBoxLayout()
        body.setSpacing(12)
        body.addWidget(self._build_control_panel(), 3)
        body.addWidget(self._build_status_panel(), 2)
        root.addLayout(body, 1)
        # The control/status area gets the expandable space.  The log remains
        # compact so it cannot squeeze and clip the jog-control buttons.
        root.addWidget(self._build_log_panel(), 0)

        self._apply_style()

        self.command_timer = QTimer(self)
        self.command_timer.setInterval(40)
        self.command_timer.timeout.connect(self._refresh_command)
        self.command_timer.start()

        self.status_timer = QTimer(self)
        self.status_timer.setInterval(200)
        self.status_timer.timeout.connect(self.refresh_backend_status)
        self.status_timer.start()

        self.append_log('info', 'UI initialized. Commands are sent only while a control is held.')

    def _build_header(self) -> QWidget:
        frame = QFrame()
        layout = QHBoxLayout(frame)
        layout.setContentsMargins(12, 8, 12, 8)

        title_box = QVBoxLayout()
        title = QLabel('RB-Y1 Universal Control UI')
        font = QFont()
        font.setPointSize(17)
        font.setBold(True)
        title.setFont(font)
        subtitle = QLabel('ROS 2 cmd_vel teleoperation · separate from the MuJoCo terminal')
        title_box.addWidget(title)
        title_box.addWidget(subtitle)
        layout.addLayout(title_box)
        layout.addStretch(1)

        self.keyboard_enable = QCheckBox('Activate keyboard control')
        self.keyboard_enable.setChecked(True)
        layout.addWidget(self.keyboard_enable)

        self.stop_button = QPushButton('MOTION STOP')
        self.stop_button.setObjectName('emergencyStop')
        self.stop_button.setMinimumHeight(48)
        self.stop_button.clicked.connect(self.emergency_stop)
        layout.addWidget(self.stop_button)
        return frame

    def _build_control_panel(self) -> QWidget:
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(10)

        layout.addWidget(self._build_speed_group())
        motion_group = self._build_motion_group()
        motion_group.setMinimumHeight(245)
        layout.addWidget(motion_group, 1)
        layout.addWidget(self._build_robot_service_group())
        return panel

    def _build_speed_group(self) -> QGroupBox:
        group = QGroupBox('Command settings')
        grid = QGridLayout(group)

        self.linear_speed = self._speed_spin(0.15, 0.01, 1.50, 0.01, ' m/s')
        self.lateral_speed = self._speed_spin(0.15, 0.01, 1.50, 0.01, ' m/s')
        self.angular_speed = self._speed_spin(0.25, 0.01, 2.50, 0.01, ' rad/s')

        grid.addWidget(QLabel('Forward / backward'), 0, 0)
        grid.addWidget(self.linear_speed, 0, 1)
        grid.addWidget(QLabel('Lateral'), 1, 0)
        grid.addWidget(self.lateral_speed, 1, 1)
        grid.addWidget(QLabel('Yaw rotation'), 2, 0)
        grid.addWidget(self.angular_speed, 2, 1)

        self.arrow_mode = QComboBox()
        self.arrow_mode.addItem('←/→ = lateral strafe (omnidirectional)', 'strafe')
        self.arrow_mode.addItem('←/→ = yaw rotation (differential style)', 'rotate')
        grid.addWidget(QLabel('Arrow-key mode'), 0, 2)
        grid.addWidget(self.arrow_mode, 0, 3, 1, 2)

        self.require_stream = QCheckBox('Allow non-zero commands only when stream is ON')
        self.require_stream.setChecked(True)
        grid.addWidget(self.require_stream, 1, 2, 1, 3)

        self.stop_on_focus_loss = QCheckBox('Stop immediately when window loses focus')
        self.stop_on_focus_loss.setChecked(True)
        grid.addWidget(self.stop_on_focus_loss, 2, 2, 1, 3)

        # The current RB-Y1 simulator interprets lateral velocity with the
        # opposite sign from the visual left/right convention used by the UI.
        # Keep this configurable so the same UI can also be reused with robots
        # whose +Y axis already matches the button labels.
        self.invert_lateral = QCheckBox('Invert lateral movement direction (RB-Y1 default)')
        self.invert_lateral.setChecked(False)
        self.invert_lateral.setToolTip(
            '체크 시 왼쪽 명령은 linear.y 음수, 오른쪽 명령은 linear.y 양수입니다.'
        )
        grid.addWidget(self.invert_lateral, 3, 2, 1, 3)
        return group

    @staticmethod
    def _speed_spin(
        value: float,
        minimum: float,
        maximum: float,
        step: float,
        suffix: str,
    ) -> QDoubleSpinBox:
        spin = QDoubleSpinBox()
        spin.setDecimals(2)
        spin.setRange(minimum, maximum)
        spin.setSingleStep(step)
        spin.setValue(value)
        spin.setSuffix(suffix)
        spin.setKeyboardTracking(False)
        return spin

    def _build_motion_group(self) -> QGroupBox:
        group = QGroupBox('Mobile base jog control')
        outer = QVBoxLayout(group)

        grid = QGridLayout()
        grid.setHorizontalSpacing(9)
        grid.setVerticalSpacing(9)

        # Compact one-line labels keep every button readable even when the
        # application window is not maximized.
        forward = self._motion_button('↑  Forward', self.ACTION_FORWARD)
        backward = self._motion_button('↓  Backward', self.ACTION_BACKWARD)
        left = self._motion_button('←  Strafe left', self.ACTION_LEFT)
        right = self._motion_button('→  Strafe right', self.ACTION_RIGHT)
        rotate_ccw = self._motion_button('↶  Rotate CCW', self.ACTION_ROTATE_CCW)
        rotate_cw = self._motion_button('↷  Rotate CW', self.ACTION_ROTATE_CW)

        center_stop = QPushButton('STOP  ·  Space')
        center_stop.setObjectName('centerStop')
        center_stop.setMinimumSize(120, 54)
        center_stop.setMaximumHeight(64)
        center_stop.clicked.connect(self.emergency_stop)

        for column in range(3):
            grid.setColumnStretch(column, 1)
        for row in range(3):
            grid.setRowMinimumHeight(row, 54)

        grid.addWidget(rotate_ccw, 0, 0)
        grid.addWidget(forward, 0, 1)
        grid.addWidget(rotate_cw, 0, 2)
        grid.addWidget(left, 1, 0)
        grid.addWidget(center_stop, 1, 1)
        grid.addWidget(right, 1, 2)
        grid.addWidget(backward, 2, 1)
        outer.addLayout(grid, 1)

        hint = QLabel(
            'Keyboard: ↑/W forward, ↓/S backward, ←/A left, →/D right, '
            'Q/E rotate, Space stop. Multiple keys can be combined.'
        )
        hint.setWordWrap(True)
        hint.setAlignment(alignment('AlignCenter'))
        outer.addWidget(hint)
        return group

    def _motion_button(self, text: str, action: str) -> QPushButton:
        button = QPushButton(text)
        button.setMinimumSize(120, 54)
        button.setMaximumHeight(64)
        button.setAutoRepeat(False)
        button.pressed.connect(lambda selected=action: self._press_action(selected))
        button.released.connect(lambda selected=action: self._release_action(selected))
        self._action_buttons[action] = button
        return button

    def _build_robot_service_group(self) -> QGroupBox:
        group = QGroupBox('RB-Y1 service controls')
        layout = QGridLayout(group)

        self.prepare_button = QPushButton('Prepare robot\nPower + Servo')
        self.prepare_button.clicked.connect(self.node.prepare_robot)

        self.power_on_button = QPushButton('Power ON')
        self.power_on_button.clicked.connect(
            lambda: self.node.request_power(True)
        )

        self.power_off_button = QPushButton('Power OFF')
        self.power_off_button.clicked.connect(
            lambda: self.node.request_power(False)
        )

        self.servo_on_button = QPushButton('Servo ON')
        self.servo_on_button.clicked.connect(
            lambda: self.node.request_servo(True)
        )

        self.servo_off_button = QPushButton('Servo OFF')
        self.servo_off_button.clicked.connect(
            lambda: self.node.request_servo(False)
        )

        self.stream_on_button = QPushButton('Stream ON')
        self.stream_on_button.clicked.connect(
            lambda: self.node.request_stream(True)
        )

        self.stream_off_button = QPushButton('Stream OFF')
        self.stream_off_button.clicked.connect(self._stream_off)

        layout.addWidget(self.prepare_button, 0, 0, 1, 2)
        layout.addWidget(self.power_on_button, 1, 0)
        layout.addWidget(self.power_off_button, 1, 1)
        layout.addWidget(self.servo_on_button, 2, 0)
        layout.addWidget(self.servo_off_button, 2, 1)
        layout.addWidget(self.stream_on_button, 3, 0)
        layout.addWidget(self.stream_off_button, 3, 1)

        return group

    def _build_status_panel(self) -> QWidget:
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(10)

        ros_group = QGroupBox('ROS connection status')
        form = QFormLayout(ros_group)
        self.namespace_value = QLabel('-')
        self.topic_value = QLabel('-')
        self.subscriber_value = QLabel('0')
        self.state_value = QLabel('unknown')
        self.stream_value = QLabel('unknown')
        self.service_value = QLabel('unknown')
        form.addRow('Namespace', self.namespace_value)
        form.addRow('cmd_vel topic', self.topic_value)
        form.addRow('Subscribers', self.subscriber_value)
        form.addRow('Robot state', self.state_value)
        form.addRow('Stream state', self.stream_value)
        form.addRow('Services', self.service_value)
        layout.addWidget(ros_group)

        command_group = QGroupBox('Current target command')
        command_layout = QGridLayout(command_group)
        self.vx_value = self._command_label()
        self.vy_value = self._command_label()
        self.wz_value = self._command_label()
        command_layout.addWidget(QLabel('linear.x'), 0, 0)
        command_layout.addWidget(self.vx_value, 0, 1)
        command_layout.addWidget(QLabel('linear.y'), 1, 0)
        command_layout.addWidget(self.vy_value, 1, 1)
        command_layout.addWidget(QLabel('angular.z'), 2, 0)
        command_layout.addWidget(self.wz_value, 2, 1)
        layout.addWidget(command_group)

        safety_group = QGroupBox('Safety behavior')
        safety_layout = QVBoxLayout(safety_group)
        safety_text = QLabel(
            '• A zero Twist is sent when controls are released.\n'
            '• Commands time out in the ROS backend.\n'
            '• Closing or deactivating the window sends STOP.\n'
            '• Stream is turned off during normal shutdown when possible.'
        )
        safety_text.setWordWrap(True)
        safety_layout.addWidget(safety_text)
        layout.addWidget(safety_group)
        layout.addStretch(1)
        return panel

    @staticmethod
    def _command_label() -> QLabel:
        label = QLabel('0.00')
        font = QFont('Monospace')
        font.setPointSize(14)
        font.setBold(True)
        label.setFont(font)
        label.setAlignment(alignment('AlignRight'))
        return label

    def _build_log_panel(self) -> QGroupBox:
        group = QGroupBox('Event log')
        layout = QVBoxLayout(group)
        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumBlockCount(500)
        self.log_view.setMinimumHeight(105)
        self.log_view.setMaximumHeight(165)
        layout.addWidget(self.log_view)
        return group

    def _apply_style(self) -> None:
        self.setStyleSheet(
            """
            QMainWindow { background: #20242b; color: #e8edf2; }
            QWidget { color: #e8edf2; font-size: 13px; }
            QGroupBox {
                border: 1px solid #4b5563;
                border-radius: 7px;
                margin-top: 12px;
                padding-top: 8px;
                font-weight: 600;
            }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; }
            QPushButton {
                background: #343b46;
                border: 1px solid #657184;
                border-radius: 6px;
                padding: 8px;
                font-weight: 600;
            }
            QPushButton:hover { background: #414b59; }
            QPushButton:pressed { background: #2c6e9b; }
            QPushButton#centerStop { background: #7f4a24; }
            QPushButton#emergencyStop { background: #9b2c2c; font-size: 14px; }
            QPushButton:disabled { color: #7d8794; background: #2a2f36; }
            QDoubleSpinBox, QComboBox, QPlainTextEdit {
                background: #171a1f;
                border: 1px solid #4b5563;
                border-radius: 4px;
                padding: 5px;
            }
            QFrame { background: #292f38; border-radius: 8px; }
            """
        )

    def _press_action(self, action: str) -> None:
        self._pressed_actions.add(action)
        self._refresh_command()

    def _release_action(self, action: str) -> None:
        self._pressed_actions.discard(action)
        self._refresh_command()

    def _stream_off(self) -> None:
        self.emergency_stop()
        self.node.request_stream(False)

    def emergency_stop(self) -> None:
        self._pressed_actions.clear()
        self.node.stop(publish_immediately=True)
        self._update_command_labels(0.0, 0.0, 0.0)

    def _refresh_command(self) -> None:
        if self._closing:
            return
        vx, vy, wz = self._calculate_command()
        snapshot = self.node.snapshot()
        if (
            self.require_stream.isChecked()
            and snapshot.services_enabled
            and snapshot.stream_enabled is not True
        ):
            vx, vy, wz = 0.0, 0.0, 0.0
        self.node.set_velocity(vx, vy, wz)
        self._update_command_labels(vx, vy, wz)

    def _calculate_command(self):
        linear = float(self.linear_speed.value())
        lateral = float(self.lateral_speed.value())
        angular = float(self.angular_speed.value())

        vx = linear * (
            int(self.ACTION_FORWARD in self._pressed_actions)
            - int(self.ACTION_BACKWARD in self._pressed_actions)
        )
        lateral_direction = (
            int(self.ACTION_LEFT in self._pressed_actions)
            - int(self.ACTION_RIGHT in self._pressed_actions)
        )
        if self.invert_lateral.isChecked():
            lateral_direction *= -1
        vy = lateral * lateral_direction
        wz = angular * (
            int(self.ACTION_ROTATE_CCW in self._pressed_actions)
            - int(self.ACTION_ROTATE_CW in self._pressed_actions)
        )
        return vx, vy, wz

    def _update_command_labels(self, vx: float, vy: float, wz: float) -> None:
        self.vx_value.setText(f'{vx:+.2f} m/s')
        self.vy_value.setText(f'{vy:+.2f} m/s')
        self.wz_value.setText(f'{wz:+.2f} rad/s')

    def refresh_backend_status(self) -> None:
        snapshot = self.node.snapshot()
        self._render_snapshot(snapshot)
        for level, message in self.node.drain_events():
            self.append_log(level, message)

    def _render_snapshot(self, snapshot: BackendSnapshot) -> None:
        self.namespace_value.setText(snapshot.namespace)
        self.topic_value.setText(snapshot.cmd_vel_topic)
        self.subscriber_value.setText(str(snapshot.cmd_vel_subscribers))
        self.state_value.setText(
            'unknown' if snapshot.control_state is None else str(snapshot.control_state)
        )
        if snapshot.stream_enabled is True:
            stream_text = 'ON'
        elif snapshot.stream_enabled is False:
            stream_text = 'OFF'
        else:
            stream_text = 'unknown'
        self.stream_value.setText(stream_text)

        if snapshot.services_enabled:
            ready = snapshot.service_ready
            service_text = (
                f'power={"ready" if ready["power"] else "wait"}, '
                f'servo={"ready" if ready["servo"] else "wait"}, '
                f'stream={"ready" if ready["stream"] else "wait"}'
            )
        else:
            service_text = 'cmd_vel-only mode'
        self.service_value.setText(service_text)

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
            widget.setEnabled(snapshot.services_enabled)

    def append_log(self, level: str, message: str) -> None:
        prefix = {'error': '[ERROR] ', 'warning': '[WARN] '}.get(level, '')
        self.log_view.appendPlainText(prefix + message)
        scrollbar = self.log_view.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

    def eventFilter(self, watched, event):  # noqa: N802 - Qt API name
        event_kind = event.type()

        if event_kind == event_type('WindowDeactivate'):
            if self.stop_on_focus_loss.isChecked():
                self.emergency_stop()
            return False

        if not self.keyboard_enable.isChecked() or not self.isActiveWindow():
            return False

        focused = QApplication.focusWidget()
        if isinstance(focused, (QAbstractSpinBox, QLineEdit, QComboBox)):
            return False

        if event_kind not in (event_type('KeyPress'), event_type('KeyRelease')):
            return False
        if event.isAutoRepeat():
            return True

        key = event.key()
        action = self._action_for_key(key)
        if key in (qt_key('Key_Space'), qt_key('Key_Escape')):
            if event_kind == event_type('KeyPress'):
                self.emergency_stop()
            return True
        if action is None:
            return False

        if event_kind == event_type('KeyPress'):
            self._press_action(action)
        else:
            self._release_action(action)
        return True

    def _action_for_key(self, key: int):
        arrow_mode = self.arrow_mode.currentData()
        mapping = {
            qt_key('Key_Up'): self.ACTION_FORWARD,
            qt_key('Key_W'): self.ACTION_FORWARD,
            qt_key('Key_Down'): self.ACTION_BACKWARD,
            qt_key('Key_S'): self.ACTION_BACKWARD,
            qt_key('Key_A'): self.ACTION_LEFT,
            qt_key('Key_D'): self.ACTION_RIGHT,
            qt_key('Key_Q'): self.ACTION_ROTATE_CCW,
            qt_key('Key_E'): self.ACTION_ROTATE_CW,
        }
        if arrow_mode == 'rotate':
            mapping[qt_key('Key_Left')] = self.ACTION_ROTATE_CCW
            mapping[qt_key('Key_Right')] = self.ACTION_ROTATE_CW
        else:
            mapping[qt_key('Key_Left')] = self.ACTION_LEFT
            mapping[qt_key('Key_Right')] = self.ACTION_RIGHT
        return mapping.get(key)

    def closeEvent(self, event: QCloseEvent) -> None:  # noqa: N802 - Qt API name
        self._closing = True
        self.emergency_stop()
        event.accept()

"""Application entry point."""

from __future__ import annotations

import signal
import sys
import time

import rclpy

from .qt_compat import QApplication, QTimer, QT_BINDING, app_exec
from .ros_backend import Rby1ControlNode
from .main_window import MainWindow


def main(args=None) -> None:
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    app.setApplicationName('RB-Y1 Universal Control UI')

    node = Rby1ControlNode()
    window = MainWindow(node)
    app.installEventFilter(window)

    spin_timer = QTimer()
    spin_timer.setInterval(10)
    spin_timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.0))
    spin_timer.start()

    signal.signal(signal.SIGINT, lambda *_: app.quit())
    signal_timer = QTimer()
    signal_timer.setInterval(250)
    signal_timer.timeout.connect(lambda: None)
    signal_timer.start()

    window.append_log('info', f'Qt binding: {QT_BINDING}')
    window.show()

    exit_code = 1
    try:
        exit_code = app_exec(app)
    finally:
        spin_timer.stop()
        signal_timer.stop()
        node.shutdown_safely(turn_stream_off=True)

        deadline = time.monotonic() + 1.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)

        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    raise SystemExit(exit_code)


if __name__ == '__main__':
    main()

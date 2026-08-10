# RB-Y1 Universal Control UI

A separate Qt window for operating the RB-Y1 MuJoCo mobile base through ROS 2.
It reuses the communication structure of `12_mobile_base_control.py`:

- `geometry_msgs/msg/Twist` publisher on `cmd_vel`
- optional `robot_power`, `robot_servo`, and `stream_control` service clients
- optional `robot_state` subscriber
- continuous 25 Hz command publication

The visual interaction follows the supplied MuJoCo Qt example:
buttons and keyboard events start motion while held and publish STOP when released.

## Intended workspace location

Place this package at exactly:

```text
<rby1_ros2_ws>/src/rby1-ros2/rby1_control_ui/
```

Do not place it inside `rby1_examples/rby1_examples/`. It is deliberately an
independent ROS 2 package so the vendor/example package does not need to be edited.

## File layout

```text
rby1_control_ui/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/rby1_control_ui
├── config/default.yaml
├── launch/control_ui.launch.py
└── rby1_control_ui/
    ├── __init__.py
    ├── qt_compat.py
    ├── ros_backend.py
    ├── main_window.py
    └── main.py
```

## Key mapping

- Up or W: forward
- Down or S: backward
- Left or A: lateral left by default
- Right or D: lateral right by default
- Q / E: rotate counterclockwise / clockwise
- Space or Escape: stop

The arrow-key mode can be changed in the UI so Left/Right rotate instead of strafe.

## Qt dependency

The code supports either PySide6 or PyQt5. Only one is required at runtime.
Installation and run verification should be performed after the package files are
placed in the workspace.

## Safe file-copy helper

After extracting the downloaded ZIP, the package can be copied without editing the
existing vendor packages:

```bash
bash /path/to/rby1_control_ui/install_into_workspace.sh /path/to/rby1_ros2_ws
```

The script checks for `src/rby1-ros2`, refuses to overwrite an existing package,
and performs no build or UI execution.

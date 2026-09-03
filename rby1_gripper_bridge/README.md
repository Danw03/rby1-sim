# RB-Y1 Gripper Bridge

`rby1_sdk.DynamixelBus`로 직접 연결되는 RB-Y1 양손 그리퍼를 ROS 2로
노출하는 독립 패키지입니다. 기존 `rby1_driver`나 `rby1_control_ui`를
수정하지 않습니다.

이 패키지는 다음 경계를 전제로 합니다.

```text
control_ui / planner  ->  backend  ->  gripper/command
                                      |
                              rby1_gripper_bridge
                                      |
                       rby1_sdk.DynamixelBus
                                      |
                             /dev/rby1_gripper
```

그리퍼 시리얼 포트는 이 bridge 프로세스 하나만 열어야 합니다. SDK의
teleoperation 예제처럼 같은 포트를 직접 여는 프로그램과 동시에 실행하지
마세요.

## ROS interface

기본 launch namespace는 `/rby1`입니다. 아래 이름은 그 namespace를 적용한
결과입니다.

| Kind | Name | Type | Contract |
|---|---|---|---|
| Subscribe | `/rby1/gripper/command` | `std_msgs/msg/Float64MultiArray` | `data=[right, left]`, 각 값은 close ratio `0.0=open`, `1.0=closed` |
| Publish | `/rby1/gripper/state` | `std_msgs/msg/Float64MultiArray` | 측정 close ratio `[right, left]`; homing 전에는 `NaN` |
| Publish | `/rby1/gripper/motor_state` | `sensor_msgs/msg/JointState` | Dynamixel 원시 위치(rad), 속도(rad/s) |
| Publish | `/rby1/gripper/ready` | `std_msgs/msg/Bool` | 통신, calibration, torque가 모두 준비되었을 때 `true` |
| Publish | `/rby1/gripper/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | calibration, 목표, 전류, 온도 요약 |
| Service | `/rby1/gripper/home` | `std_srvs/srv/Trigger` | 양쪽 full-travel homing 및 torque enable |
| Service | `/rby1/gripper/torque_enable` | `std_srvs/srv/SetBool` | 양쪽 torque on/off |

명령은 길이 2의 유한한 값만 받고, 범위 밖 값을 clamp하지 않고 거부합니다.
Homing 또는 저장된 calibration 없이 position command를 보내는 것도
거부합니다. 양손을 한 메시지로 보내므로 handover처럼 동시성이 필요한
명령에서 한쪽만 갱신되는 중간 상태가 생기지 않습니다.

## Build

ROS 2를 사용하는 Python 환경에 공식 SDK를 먼저 설치합니다.

```bash
python3 -m pip install "rby1-sdk>=0.10.0"
colcon build --symlink-install --packages-select rby1_gripper_bridge
source install/setup.bash
```

## Hardware run

먼저 `/dev/rby1_gripper`가 존재하고 현재 사용자에게 읽기/쓰기 권한이 있는지
확인합니다. Bridge는 안전을 위해 기본값으로 자동 homing하지 않습니다.

Terminal 1:

```bash
ros2 launch rby1_gripper_bridge gripper_bridge.launch.py
```

Terminal 2의 가벼운 interactive debug node:

```bash
ros2 run rby1_gripper_bridge gripper_debug_controller \
  --ros-args -r __ns:=/rby1
```

프롬프트에서 `home`을 실행하기 전에 양쪽 그리퍼의 전체 이동 경로를
비우세요. Homing은 양 끝의 hard stop을 찾기 위해 양쪽 그리퍼를 완전히
왕복시킵니다. 완료 후 다음 명령으로 바로 확인할 수 있습니다.

```text
open both
close right
left 0.5
set 0.25 0.75
state
torque off
```

토픽만으로도 명령할 수 있습니다.

```bash
ros2 topic pub --once /rby1/gripper/command \
  std_msgs/msg/Float64MultiArray "{data: [1.0, 0.0]}"
```

## ROS-only loopback test

실기 없이 backend/topic 배선을 검증할 때는 mock 모드를 사용합니다. 이
모드는 in-process 상태만 흉내 내며 MuJoCo gripper를 움직이지 않습니다.

Terminal 1:

```bash
ros2 launch rby1_gripper_bridge gripper_bridge.launch.py \
  mock_hardware:=true auto_home:=true
```

Terminal 2:

```bash
ros2 run rby1_gripper_bridge gripper_debug_controller \
  --ros-args -r __ns:=/rby1
```

## Reusing calibration

성공한 homing 결과는 node log와 diagnostics에 `min=[right, left]`,
`max=[right, left]`로 표시됩니다. 값을 `config/default.yaml`을 복사한 별도
운영 설정에 넣으면 다음 startup부터 full-travel 동작을 생략할 수 있습니다.

```yaml
/**:
  ros__parameters:
    use_saved_calibration: true
    calibration_min_rad: [-1.23, -1.20]
    calibration_max_rad: [1.15, 1.18]
    auto_enable_torque: true
```

Calibration 값은 예시를 복사하지 말고 반드시 해당 하드웨어에서 측정한 값을
사용하세요.

## Important parameters

- `device_name`: 빈 문자열이면 SDK의 `GripperDeviceName`을 사용합니다.
- `right_motor_id`, `left_motor_id`: SDK 예제와 같은 기본값 `0`, `1`입니다.
- `closed_at_minimum`: 기본 `[true, true]`이며 SDK 예제의 방향과 같습니다.
- `position_torque_limit`: 기본 `5.0`; upstream SDK 예제 값이지만 실제
  그리퍼와 payload에 맞게 검증해야 합니다.
- `homing_torque`: 기본 `0.3`.
- `endpoint_margin_ratio`: 양 끝 hard stop을 피할 운영 margin이며 기본은 SDK
  동작과 같은 `0.0`입니다.
- `disable_torque_on_shutdown`: 정상 종료 시 torque를 해제하며 기본
  `true`입니다.

## Backend integration note

토픽 명령 자체는 fire-and-forget입니다. 이후 backend는 command를 publish한
뒤 `/rby1/gripper/state`가 목표 tolerance 안에 들어오는지, `/ready`가 계속
true인지, timeout이 지나지 않았는지를 확인한 다음 planner에 성공/실패를
반환해야 합니다. 이 완료 판정과 task sequencing은 hardware bridge가 아니라
backend의 책임으로 남겨 두었습니다.

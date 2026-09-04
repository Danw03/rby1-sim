# RBY1 Gripper Bridge

RBY1 기본 양손 그리퍼를 `rby1_sdk.DynamixelBus`로 제어하고 ROS 2
인터페이스로 노출하는 실물 전용 패키지입니다. 기존 `rby1_driver`,
`rby1_control_ui` 등 다른 패키지는 수정하지 않습니다.

```text
control_ui / planner
        |
future command/backend node
        |
/rby1/gripper/command
        |
rby1_gripper_bridge (U-PC)
        |
rby1_sdk.DynamixelBus
        |
/dev/rby1_gripper -> U2D2/RS485 -> Dynamixel ID 0, 1
```

이 노드는 `/dev/rby1_gripper`가 보이는 U-PC에서 실행해야 합니다.
그리퍼 command/state는 `rby1_driver`나 robot RPC를 거치지 않고 이
serial device로 직접 오갑니다. 동일한 serial device를 다른 프로그램과
동시에 열면 안 됩니다.

단, **통신 경로와 전원 경로는 별개**입니다. 그리퍼 모터에는 양쪽
tool-flange의 12 V 출력이 필요합니다. 전원이 꺼져 있다면 기존 로봇 기동
절차나 `rby1_driver`의 robot/tool-flange power service로 먼저 켜야
합니다. Control UI 자체는 필수 조건이 아닙니다.

## ROS interface

기본 launch namespace는 `/rby1`입니다. 모든 2원소 배열의 순서는
`[right, left]`입니다.

| Kind | Name | Type | Contract |
|---|---|---|---|
| Subscribe | `/rby1/gripper/command` | `std_msgs/msg/Float64MultiArray` | `data=[right, left]`, `0.0=open`, `1.0=closed` |
| Publish | `/rby1/gripper/state` | `std_msgs/msg/Float64MultiArray` | encoder로 계산한 실제 close ratio |
| Publish | `/rby1/gripper/motor_state` | `sensor_msgs/msg/JointState` | 실제 위치(rad), 속도(rad/s) |
| Publish | `/rby1/gripper/ready` | `std_msgs/msg/Bool` | command를 받을 준비가 되었는지 표시 |
| Publish | `/rby1/gripper/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 장치, SDK, 통신, calibration, 전류, 온도 |
| Service | `/rby1/gripper/home` | `std_srvs/srv/Trigger` | 양쪽 hard stop을 찾아 calibration |
| Service | `/rby1/gripper/torque_enable` | `std_srvs/srv/SetBool` | 양쪽 motor torque on/off |

command는 길이가 정확히 2이고 각 값이 유한한 0~1 범위일 때만 받습니다.
`ready=true`일 때만 command가 실행됩니다.

`ready=true` 조건은 다음을 모두 만족하는 것입니다.

- serial port open 및 baud 설정 성공
- 오른쪽/왼쪽 motor ping 성공
- 유효한 calibration 존재
- 양쪽 torque enable
- homing 중이 아님
- 마지막 통신 상태가 정상

## Build

ROS 환경과 이 workspace가 사용하는 SDK 환경을 적용한 뒤 빌드합니다.

```bash
export RBY1_SDK_PATH=/root/sdk/rby1-sdk
source /opt/ros/humble/setup.bash
cd ~/rby1_ros2_ws

colcon build --symlink-install --packages-select rby1_gripper_bridge
source install/setup.bash
```

전체 workspace 빌드가 필요하면 다음을 사용합니다.

```bash
colcon build --symlink-install --cmake-clean-cache
source install/setup.bash
```

## 실물 검증 전 준비

Homing은 양쪽 그리퍼를 낮은 torque로 양 끝까지 완전히 왕복시킵니다.
처음 검증할 때는 물체와 payload를 잡지 말고, 손과 케이블을 이동 범위에서
치우며 비상정지 장치를 즉시 누를 수 있게 준비합니다.

그리퍼 serial port는 bridge 한 프로세스만 사용해야 합니다. SDK 예제,
teleoperation 프로그램 또는 이전 bridge 프로세스를 동시에 실행하지
마십시오.

### 1. 로봇과 그리퍼 전원 확인

연구실의 기존 실물 기동 절차로 로봇 전원과 양쪽 tool-flange 12 V가 이미
켜져 있다면 이 단계에서는 상태만 확인합니다. `rby1_driver`를 이용해
전원을 켜는 경우 먼저 실제 로봇 주소로 driver를 실행한 뒤 service 이름을
찾습니다.

```bash
ros2 service list | grep tool_flange_power
ros2 service list | grep robot_power
```

기본 namespace가 `/rby1`인 경우의 예시는 다음과 같습니다. 실제
`ros2 service list` 결과가 `/tool_flange_power`처럼 다르면 그 이름을
사용합니다.

```bash
# 48 V가 이미 켜져 있다면 첫 호출은 생략합니다.
ros2 service call /rby1/robot_power rby1_msgs/srv/StateOnOff \
  "{state: true, parameters: '48v', value: 0.0}"

ros2 service call /rby1/tool_flange_power rby1_msgs/srv/StateOnOff \
  "{state: true, parameters: '12v', value: 0.0}"
```

tool-flange state publish가 활성화되어 있으면 양쪽 출력 전압도 확인합니다.

```bash
ros2 topic echo --once /rby1/tool_flange/right
ros2 topic echo --once /rby1/tool_flange/left
```

`rby1_driver`는 여기서 전원 설정/모니터링에만 사용됩니다. 이후 그리퍼
명령은 bridge가 U-PC의 Dynamixel serial로 직접 보냅니다. 따라서 driver와
bridge를 함께 실행해도 되지만, SDK gripper 예제처럼
`/dev/rby1_gripper`를 여는 다른 프로세스는 함께 실행하면 안 됩니다.

### 2. 장치와 Python SDK 확인

U-PC에서 실행합니다.

```bash
ls -l /dev/rby1_gripper
readlink -f /dev/rby1_gripper
python3 -c "import rby1_sdk; print(rby1_sdk.__file__); print(rby1_sdk.upc.GripperDeviceName); print(rby1_sdk.DynamixelBus)"
```

기대 결과:

- `/dev/rby1_gripper`가 실제 tty 장치를 가리킴
- Python import 성공
- 기본 device name이 `/dev/rby1_gripper`
- `DynamixelBus` type이 출력됨

장치를 다른 프로세스가 사용 중인지 확인할 수 있습니다.

```bash
fuser -v /dev/rby1_gripper
```

bridge를 실행하기 전에는 불필요한 PID가 없어야 합니다. `fuser`가
설치되지 않았다면 이 검사는 생략하고, 관련 예제와 node를 모두 종료합니다.

Docker 안에서 bridge를 실행한다면 컨테이너 시작 시 실제 장치를 넘겨야
합니다.

```text
--device=/dev/rby1_gripper:/dev/rby1_gripper
```

컨테이너 안에서도 `ls -l /dev/rby1_gripper`와 SDK import가 성공해야
합니다. 일반 사용자로 실행할 때는 해당 장치의 group 권한도 필요합니다.

첫 검증에서는 bridge와 debug controller를 같은 컨테이너에서 실행하는 것이
가장 단순합니다. 서로 다른 컨테이너에서 실행한다면 두 컨테이너의
`ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION`, DDS 설정과 network mode가
일치해야 합니다.

```bash
printenv ROS_DOMAIN_ID
printenv RMW_IMPLEMENTATION
ros2 node info /rby1/gripper_bridge
ros2 topic info -v /rby1/gripper/command
```

노드는 보이는데 CLI graph가 서로 다르면 각 환경에서
`ros2 daemon stop` 후 `ros2 daemon start`를 실행해 cache를 새로 만든
뒤 다시 확인합니다. 그래도 다르면 bridge와 debug controller를 같은
컨테이너에서 먼저 검증해 DDS/container 문제와 하드웨어 문제를 분리합니다.

### 3. Bridge 시작: 자동 homing 금지

첫 검증에서는 `auto_home=false`를 유지합니다.

```bash
source /opt/ros/humble/setup.bash
source ~/rby1_ros2_ws/install/setup.bash

ros2 launch rby1_gripper_bridge gripper_bridge.launch.py \
  auto_home:=false
```

정상적으로 port open, baud 설정 및 두 motor ping이 끝나면 launch terminal에
다음 정보가 출력됩니다.

```text
Dynamixel device=/dev/rby1_gripper
baud=2000000
IDs=(0, 1)
```

이 단계에서 calibration을 저장해 두지 않았다면 `ready=false`와
`Calibration required`가 정상입니다.

노드와 실제 측정 상태를 확인합니다.

```bash
ros2 node list
ros2 topic echo --once /rby1/gripper/ready
ros2 topic echo --once /rby1/gripper/diagnostics
ros2 topic echo --once /rby1/gripper/motor_state
```

정상적인 초기 diagnostics의 핵심 값은 다음과 같습니다.

```text
initialized: true
healthy: true
torque_enabled: false
message: Calibration required
motor_ids_right_left: [0, 1]
state_source: dynamixel_measurement
```

여기서 bridge가 종료된다면 launch terminal의 메시지로 원인을 구분합니다.

| 오류 | 우선 확인할 항목 |
|---|---|
| `rby1_sdk is not installed` | launch에 사용된 Python 환경과 SDK 설치 |
| `failed to open the gripper port` | device 경로, 권한, Docker device mount, 중복 사용 |
| `failed to set ... baud rate` | U2D2/USB 상태와 baud 설정 |
| `IDs did not respond` | 양쪽 tool-flange 12 V, RS485 배선, ID 0/1, baud 2000000 |

### 4. 한 번만 homing

양쪽 이동 범위가 완전히 비었는지 다시 확인하고 호출합니다.

```bash
ros2 service call /rby1/gripper/home std_srvs/srv/Trigger "{}"
```

Homing 중에는 `ready=false`입니다. 서비스가 끝날 때까지 기다립니다.
성공 응답에는 실제 encoder endpoint가 오른쪽/왼쪽 순서로 나옵니다.

```text
success: true
message: min=[right_min, left_min], max=[right_max, left_max]
```

다음도 확인합니다.

```bash
ros2 topic echo --once /rby1/gripper/ready
ros2 topic echo --once /rby1/gripper/diagnostics
```

성공 후에는 `ready=true`, `healthy=true`, `torque_enabled=true`이고
diagnostics에 calibration min/max가 표시되어야 합니다.

Homing이 timeout되거나 한쪽만 움직이면 torque 값을 먼저 높이지 마십시오.
즉시 bridge를 종료하고 전원, ID, 배선, 양쪽 기구 걸림 및 encoder 변화를
확인합니다.

## Debug controller로 단계적 동작 확인

새 terminal에서 실행합니다.

```bash
source /opt/ros/humble/setup.bash
source ~/rby1_ros2_ws/install/setup.bash

ros2 run rby1_gripper_bridge gripper_debug_controller \
  --ros-args -r __ns:=/rby1
```

먼저 현재 상태를 확인합니다.

```text
state
```

`published close ratios`는 ROS command를 보냈다는 뜻일 뿐, 모터가 목표에
도달했다는 뜻은 아닙니다. 매 단계마다 `state`, `motor_state`,
`diagnostics`의 실제 측정값을 함께 봐야 합니다.

디버깅할 때는 다음 세 단계를 구분합니다.

| 관측 결과 | 확인된 범위 |
|---|---|
| debug controller에 `published close ratios` 출력 | debug node가 ROS publish 호출 |
| diagnostics의 `target_close_ratio_right_left` 변경 | bridge가 command를 수락하고 SDK position write 호출 |
| `state`와 `motor_state.position` 변경 | Dynamixel encoder 기준으로 실제 모터가 이동 |

두 번째까지만 되고 세 번째가 안 되면 ROS topic 문제가 아닙니다. 이 경우
`torque_enabled`, tool-flange 12 V, Dynamixel current/temperature, 기구
걸림을 확인합니다. SDK의 group sync write는 명령 전송 함수이므로
`published` 로그 자체는 물리적 도달 확인이 아닙니다.

기본 설정은 작은 encoder 쪽을 closed로 해석하므로, homing 직후에는 보통
closed 쪽에 있습니다. 먼저 10%만 움직여 좌우 방향과 매핑을 확인합니다.

```text
set 0.90 1.00
state
set 1.00 1.00
state

set 1.00 0.90
state
set 1.00 1.00
state
```

기대 동작:

- `set 0.90 1.00`: 오른쪽만 closed에서 약간 open 방향으로 이동
- `set 1.00 0.90`: 왼쪽만 closed에서 약간 open 방향으로 이동

반대 손이 움직이면 motor ID 매핑을 확인합니다. 열림/닫힘 방향이 반대라면
`closed_at_minimum`을 해당 손에 맞게 변경한 뒤 calibration 의미를 다시
검증합니다.

좌우와 방향이 맞은 뒤에만 범위를 단계적으로 확대합니다.

```text
set 0.75 0.75
state
set 0.50 0.50
state
set 0.25 0.25
state
set 0.00 0.00
state
set 0.25 0.25
set 0.50 0.50
set 0.75 0.75
set 1.00 1.00
state
```

다른 terminal에서 raw 측정값을 계속 볼 수 있습니다.

```bash
ros2 topic echo /rby1/gripper/motor_state
```

```bash
ros2 topic echo /rby1/gripper/diagnostics
```

다음이 보이면 즉시 명령을 중단하고 torque를 끕니다.

- 명령과 반대 방향으로 이동
- 반대쪽 gripper가 이동
- 기구 간섭, 이상음, 진동 또는 한쪽 정지
- 상태값이 변하지 않는데 전류가 계속 증가
- 온도가 빠르게 상승하거나 통신이 unhealthy로 변경

```bash
ros2 service call /rby1/gripper/torque_enable \
  std_srvs/srv/SetBool "{data: false}"
```

정상적인 `Ctrl-C` 종료도 기본 설정에서는 양쪽 torque를 해제합니다.

## Calibration 저장과 재사용

성공한 homing 응답의 endpoint를 별도 운영 YAML에 저장하면 다음 실행부터
full-travel homing을 생략할 수 있습니다. 아래 숫자는 형식 예시이며 실제
로봇에 그대로 사용하면 안 됩니다.

```yaml
/**:
  ros__parameters:
    use_saved_calibration: true
    calibration_min_rad: [-1.23, -1.20]
    calibration_max_rad: [1.15, 1.18]
    closed_at_minimum: [true, true]
    endpoint_margin_ratio: 0.0
    auto_enable_torque: true
```

```bash
ros2 launch rby1_gripper_bridge gripper_bridge.launch.py \
  config:=/absolute/path/to/hardware.yaml auto_home:=false
```

저장된 calibration을 사용할 때 bridge는 현재 encoder 위치를 먼저 읽고 그
위치에서 holding을 시작한 뒤 `ready=true`가 됩니다. Gripper 기구나 motor
설정, ID, 조립 방향이 바뀌었다면 저장값을 재사용하지 말고 다시 homing합니다.

`endpoint_margin_ratio`를 0보다 크게 설정하면 운영 command가 hard stop에
직접 닿지 않게 할 수 있습니다. 값을 바꾸면 유효 stroke와 0/1 의미를 다시
검증해야 합니다.

## 향후 command/backend node 연결

향후 분리할 backend node는 `/rby1/gripper/command`에 항상
`[right, left]` 전체 목표를 publish합니다. 성공 여부는 publish 자체가
아니라 다음 조건으로 판단해야 합니다.

- `/rby1/gripper/ready == true`
- `/rby1/gripper/state`가 목표 tolerance 안에 도달
- timeout이 지나지 않음
- diagnostics가 error 상태가 아님

종료나 fault 처리 시에는 `/rby1/gripper/torque_enable`을 사용해 torque를
해제할 수 있습니다.

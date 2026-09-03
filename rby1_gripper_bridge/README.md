# RBY1 Gripper Bridge

RBY1 양손 그리퍼를 하나의 ROS 2 인터페이스로 노출하는 독립 패키지입니다.
기존 `rby1_driver`, `rby1_control_ui` 등 다른 패키지는 수정하지 않습니다.

두 실행 backend를 제공합니다.

- `mujoco` (기본값): 공식 RBY1 MuJoCo 서버의
  `rb.api.GripperCommandService`에 gRPC로 연결합니다. 기본 주소는
  `127.0.0.1:50051`입니다.
- `dynamixel`: 실물 RBY1 그리퍼의 `/dev/rby1_gripper`를
  `rby1_sdk.DynamixelBus`로 직접 제어합니다.

```text
control_ui / planner  ->  future backend  ->  /rby1/gripper/command
                                                |
                                      rby1_gripper_bridge
                                       /                \
                     MuJoCo gRPC :50051                  Dynamixel serial
```

MuJoCo RPC가 받는 `right`/`left`의 position, velocity, force 단위는
각각 0~100입니다. ROS에서는 backend와 무관하게 close ratio
`0.0=open`, `1.0=closed`를 사용하고 bridge가 position percent로
변환합니다.

## ROS interface

기본 launch namespace는 `/rby1`입니다.

| Kind | Name | Type | Contract |
|---|---|---|---|
| Subscribe | `/rby1/gripper/command` | `std_msgs/msg/Float64MultiArray` | `data=[right, left]`, `0.0=open`, `1.0=closed` |
| Publish | `/rby1/gripper/state` | `std_msgs/msg/Float64MultiArray` | `[right, left]`; 실물은 측정값, MuJoCo는 양쪽 RPC가 성공한 마지막 목표값 |
| Publish | `/rby1/gripper/motor_state` | `sensor_msgs/msg/JointState` | 실물 Dynamixel의 위치(rad), 속도(rad/s); MuJoCo에서는 발행하지 않음 |
| Publish | `/rby1/gripper/ready` | `std_msgs/msg/Bool` | 명령을 받을 준비가 되었는지 표시 |
| Publish | `/rby1/gripper/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | backend, 연결 상태, 상태 출처, 목표, 실물 전류/온도 |
| Service | `/rby1/gripper/home` | `std_srvs/srv/Trigger` | MuJoCo는 RPC 재초기화, 실물은 full-travel homing |
| Service | `/rby1/gripper/torque_enable` | `std_srvs/srv/SetBool` | 실물 torque on/off; MuJoCo는 disable RPC가 없어 `false` 요청을 거부 |

명령은 길이가 정확히 2이고 각 값이 유한한 0~1 범위일 때만 받습니다.
양손 목표를 한 ROS 메시지로 전달하므로 상위 backend는 항상
`[right, left]` 전체 값을 publish하면 됩니다.

주의: 공식 `GripperCommandService`에는 상태 조회 RPC가 없습니다. 따라서
MuJoCo의 `/gripper/state`는 화면에서 측정한 관절값이 아니라 서버가 오류 없이
받은 마지막 목표값입니다. 실제 동작 여부는 MuJoCo 화면으로 확인하고,
diagnostics의 `state_source=last_rpc_accepted_target`도 함께 확인하십시오.

## Build

현재 사용 중인 SDK 경로와 ROS 환경을 먼저 적용합니다.

```bash
export RBY1_SDK_PATH=/root/sdk/rby1-sdk
source /opt/ros/humble/setup.bash
cd ~/rby1_ros2_ws

# grpc import가 실패할 때만 설치
python3 -c "import grpc; print(grpc.__version__)"
# sudo apt install python3-grpcio

colcon build --symlink-install --packages-select rby1_gripper_bridge
source install/setup.bash
```

전체 workspace를 다시 빌드하려면 마지막 build 명령만 다음처럼 바꿉니다.

```bash
colcon build --symlink-install --cmake-clean-cache
```

## MuJoCo 실행 및 화면 검증

연구실 PC에서 기존 MuJoCo 시뮬레이터를 먼저 띄운 뒤 50051 포트를
확인합니다.

```bash
ss -ltn | grep ':50051'
```

Terminal 1에서 bridge를 실행합니다. `mujoco`가 기본 backend라 첫 명령은
짧게 써도 됩니다.

```bash
source /opt/ros/humble/setup.bash
source ~/rby1_ros2_ws/install/setup.bash
ros2 launch rby1_gripper_bridge gripper_bridge.launch.py
```

시뮬레이터가 다른 PC에 있으면 주소만 바꿉니다.

```bash
ros2 launch rby1_gripper_bridge gripper_bridge.launch.py \
  backend:=mujoco robot_address:=192.168.x.x:50051
```

Terminal 2에서 가벼운 interactive debug node를 실행합니다.

```bash
source /opt/ros/humble/setup.bash
source ~/rby1_ros2_ws/install/setup.bash
ros2 run rby1_gripper_bridge gripper_debug_controller \
  --ros-args -r __ns:=/rby1
```

다음 명령을 한 줄씩 입력하면 MuJoCo 화면의 양쪽 손가락이 실제로
움직여야 합니다.

```text
open both
close right
left 0.5
set 0.25 0.75
close both
open both
state
```

토픽만으로도 바로 시험할 수 있습니다.

```bash
ros2 topic pub --once /rby1/gripper/command \
  std_msgs/msg/Float64MultiArray "{data: [1.0, 0.0]}"

ros2 topic echo --once /rby1/gripper/ready
ros2 topic echo --once /rby1/gripper/diagnostics
```

연결이 끊겨 `/ready=false`가 되면 시뮬레이터와 50051 포트를 복구한 뒤
다음 서비스로 gRPC 연결과 양쪽 그리퍼를 재초기화합니다.

```bash
ros2 service call /rby1/gripper/home std_srvs/srv/Trigger "{}"
```

## 실물 검증 절차

실물에서는 그리퍼 시리얼 포트를 이 bridge 하나만 열어야 합니다. SDK의
teleoperation 예제나 다른 그리퍼 프로그램을 동시에 실행하지 마십시오.

1. 비상정지 버튼을 바로 누를 수 있게 두고, 양쪽 그리퍼의 전체 이동 경로와
   내부를 비웁니다. 처음에는 물체나 payload를 잡지 않습니다.
2. 장치와 SDK를 확인합니다.

   ```bash
   ls -l /dev/rby1_gripper
   python3 -c "import rby1_sdk; print('rby1_sdk OK')"
   ```

3. Terminal 1에서 자동 homing 없이 실물 backend를 시작합니다.

   ```bash
   source /opt/ros/humble/setup.bash
   source ~/rby1_ros2_ws/install/setup.bash
   ros2 launch rby1_gripper_bridge gripper_bridge.launch.py \
     backend:=dynamixel auto_home:=false
   ```

4. 다른 terminal에서 초기 상태를 확인합니다. 이 시점의 `ready=false`와
   `Calibration required`는 정상입니다.

   ```bash
   source /opt/ros/humble/setup.bash
   source ~/rby1_ros2_ws/install/setup.bash
   ros2 topic echo --once /rby1/gripper/ready
   ros2 topic echo --once /rby1/gripper/diagnostics
   ```

5. 손을 치운 상태에서 한 번만 homing합니다. 양쪽 그리퍼가 낮은 torque로
   양 끝 hard stop을 찾기 위해 완전히 왕복합니다. 이상음, 비대칭 걸림,
   과도한 변형이 보이면 즉시 비상정지하거나 bridge를 종료합니다.

   ```bash
   ros2 service call /rby1/gripper/home std_srvs/srv/Trigger "{}"
   ```

6. 서비스 응답의 `min=[right,left]`, `max=[right,left]`를 기록하고
   `ready=true`인지 확인합니다. 그다음 작은 단계부터 시험합니다.

   ```bash
   ros2 topic pub --once /rby1/gripper/command \
     std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0]}"
   ros2 topic pub --once /rby1/gripper/command \
     std_msgs/msg/Float64MultiArray "{data: [0.1, 0.1]}"
   ros2 topic pub --once /rby1/gripper/command \
     std_msgs/msg/Float64MultiArray "{data: [0.25, 0.25]}"
   ```

7. `/rby1/gripper/state`가 목표를 따라가는지, diagnostics의 전류와 온도가
   비정상적으로 증가하지 않는지 확인한 뒤에만 0.5, 0.75, 1.0으로
   단계적으로 올립니다. 오른쪽과 왼쪽도 따로 검증합니다.
8. 검증을 마치면 torque를 해제합니다. `Ctrl-C`로 정상 종료해도 기본 설정은
   torque를 해제합니다.

   ```bash
   ros2 service call /rby1/gripper/torque_enable \
     std_srvs/srv/SetBool "{data: false}"
   ```

## 실물 calibration 재사용

성공한 homing 결과를 별도 운영 YAML에 저장하면 다음 실행부터 hard stop
왕복을 생략할 수 있습니다. 아래 숫자는 형식 예시일 뿐 복사해서 사용하면
안 됩니다.

```yaml
/**:
  ros__parameters:
    backend: "dynamixel"
    use_saved_calibration: true
    calibration_min_rad: [-1.23, -1.20]
    calibration_max_rad: [1.15, 1.18]
    auto_enable_torque: true
```

```bash
ros2 launch rby1_gripper_bridge gripper_bridge.launch.py \
  backend:=dynamixel config:=/absolute/path/to/hardware.yaml
```

`endpoint_margin_ratio`를 0보다 조금 크게 두면 실제 운용 목표가 hard stop에
직접 닿지 않게 할 수 있습니다. 값 변경 후에는 open/close 의미와 유효
stroke를 다시 검증하십시오.

## 향후 backend 패키지 연결

향후 분리할 backend 노드는 `/rby1/gripper/command`를 publish하고,
`/rby1/gripper/ready`, `/rby1/gripper/state`, diagnostics를 확인해 timeout과
완료 여부를 planner에 반환하면 됩니다. MuJoCo state는 accepted target이므로
시뮬레이션 단계에서는 시간 제한과 화면/작업 결과를 함께 사용하고, 실물
단계에서는 측정 state tolerance로 완료를 판정하는 것이 안전합니다.

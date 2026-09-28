# Withus ROS 2 Driver

Withus 플랫폼의 CAN 피드백을 수신하고 ROS 2 토픽으로 제공하는 C++17 드라이버입니다.
Linux SocketCAN으로 `0x301`, `0x303`, `0x304`의 차량 상태와 `0x10A`의 조종기 A 입력을 해석하며, ID별 수신 여부와 데이터 유효성을 함께 발행합니다.

기본 실행은 `hardware_drive`와 `enable_tx=true`입니다. `withus_bringup.launch.py` 하나로 드라이버를 실행하며, 시작부터 `0x501` 요청 1을 계속 송신합니다. 조종기 A로 모드를 선택하고 client가 `DriveCommand`로 속도·조향·제동·기어 목표를 보냅니다. 새 client 명령을 받기 전에는 구동 프레임 `0x502/503/504`를 보내지 않습니다.

공개 프로필은 실차용 `hardware_drive`와 가상 CAN 검증용 `vcan_reference` 두 가지입니다. 상태 수신만 필요하면 같은 launch에 `enable_tx:=false`를 지정합니다. 일반 주행과 정지 목표는 client가 결정하고, 드라이버는 명령 검증·변환·송신과 명령 폐기를 담당합니다.

상태 모니터와 키보드 제어 예제의 launch는 `withus_examples`에 있습니다. 이 launch들은 client만 실행하며, 이미 실행한 드라이버를 사용합니다.

## 패키지 구성

| 패키지 | 역할 |
| --- | --- |
| [withus_driver](withus_driver/) | SocketCAN 통신, 프레임별 디코딩, ID별 상태 저장, ROS 상태 발행 |
| [withus_msgs](withus_msgs/) | 차량·드라이버 상태, 주행·장치 명령 메시지와 제어 서비스 |
| [withus_bringup](withus_bringup/) | 드라이버 launch, CAN 설정 스크립트, 실행 의존성 |
| [withus_examples](withus_examples/README.md) | Python 상태 모니터 및 키보드 주행 제어 예제 |

```text
withus_driver_ros2/
├── README.md
├── docs/withus_CAN_datasheet.pdf
├── withus_driver/
│   ├── include/withus_driver/
│   │   ├── can_frame.hpp
│   │   ├── socket_can_transport.hpp
│   │   ├── withus_protocol.hpp
│   │   ├── vehicle_state.hpp
│   │   ├── control_manager.hpp
│   │   ├── hardware_drive_controller.hpp
│   │   └── withus_driver_node.hpp
│   ├── src/
│   │   ├── socket_can_transport.cpp
│   │   ├── can_connection_check.cpp
│   │   ├── withus_protocol.cpp
│   │   ├── withus_command_protocol.cpp
│   │   ├── control_manager.cpp
│   │   ├── hardware_drive_controller.cpp
│   │   ├── withus_driver_node.cpp
│   │   └── main.cpp
│   ├── CMakeLists.txt
│   └── package.xml
├── withus_msgs/
│   ├── msg/VehicleStatus.msg
│   ├── msg/DriverStatus.msg
│   ├── msg/DriveCommand.msg
│   ├── msg/BodyCommand.msg
│   ├── srv/SetControl.srv
│   ├── CMakeLists.txt
│   └── package.xml
├── withus_bringup/
│   ├── launch/withus_bringup.launch.py
│   ├── config/withus_driver.yaml
│   ├── scripts/setup_can.sh
│   ├── CMakeLists.txt
│   └── package.xml
└── withus_examples/
    ├── launch/status_monitor.launch.py
    ├── launch/keyboard_teleop.launch.py
    ├── withus_examples/status_monitor.py
    ├── withus_examples/keyboard_teleop.py
    ├── README.md
    ├── setup.py
    ├── setup.cfg
    └── package.xml
```

## 준비 및 빌드

Linux, ROS 2 Humble, C++17을 기준으로 합니다. ROS 2, `colcon`, `rosdep` 설치와 초기 설정을 마친 환경에서 실행합니다.

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_workspace/withus_ws
rosdep install --from-paths src --ignore-src --rosdistro humble -y
colcon build --packages-up-to withus_bringup withus_examples
source install/setup.bash
```

`withus_msgs`를 포함한 의존 패키지부터 빌드합니다. 메시지 정의를 수정했다면 관련 패키지를 다시 빌드하고, 실행할 각 터미널에서 새 `install/setup.bash`를 적용하세요.

`withus_bringup/package.xml`에는 `iproute2`, `can-utils` 실행 의존성이 선언돼 있습니다. `rosdep install`로 CAN 설정용 `ip`와 수신 확인용 `candump` 등의 도구를 준비할 수 있습니다.

## CAN 인터페이스 준비

아래 명령은 CAN 인터페이스 `can0`, 통신 속도 `500000 bit/s`를 기준으로 합니다. 실제 연결 환경에 맞게 지정하세요.

```bash
ip -details link show type can
ros2 run withus_bringup setup_can.sh can0 500000
```

[setup_can.sh](withus_bringup/scripts/setup_can.sh)는 인자를 생략하면 `can0`, `500000`을 사용합니다. 인터페이스 확인 → `down` → bitrate 설정 → `up` → 상세 상태·통계 출력 순서이며, 설정 시 `sudo`를 사용합니다.

인터페이스를 내리므로 드라이버 실행 전에 사용하세요. 이미 원하는 속도로 활성화돼 있으면 재설정할 필요가 없습니다. 중간 명령이 실패하면 중단하며 이전 설정을 자동 복구하지 않습니다. 가상 CAN인 `vcan`에는 이 bitrate 설정 스크립트를 사용하지 않습니다.

원시 프레임은 `candump can0`로 확인하고 `Ctrl+C`로 종료합니다. 드라이버와 launch는 bitrate 설정이나 인터페이스 활성화를 수행하지 않습니다.

## 실행과 상태 확인

### Launch 실행

```bash
ros2 launch withus_bringup withus_bringup.launch.py
```

기본값은 `can_interface=can0`, `control_profile=hardware_drive`, `enable_tx=true`입니다. CAN 인터페이스는 실행 전에 준비해야 합니다. 수신만 하려면 다음처럼 송신을 끕니다.

```bash
ros2 launch withus_bringup withus_bringup.launch.py enable_tx:=false
```

가상 CAN 프로필은 커널 링크 종류가 `vcan`인 인터페이스에서만 송신합니다.

```bash
ros2 launch withus_bringup withus_bringup.launch.py \
  can_interface:=vcan0 control_profile:=vcan_reference
```

노드 이름은 `/withus/withus_driver`입니다. 다른 터미널에서 워크스페이스 환경을 적용한 뒤 확인합니다.

```bash
cd ~/ros2_workspace/withus_ws
source install/setup.bash
ros2 topic echo /withus/vehicle_status --once
ros2 topic hz /withus/vehicle_status
```

토픽과 메시지 필드의 의미는 아래 [ROS 토픽 인터페이스](#ros-토픽-인터페이스)와 [VehicleStatus 메시지 필드](#vehiclestatus-메시지-필드)를 참고하세요.

| 시작 파라미터 / launch 인자 | 타입 | 기본값 | 의미 |
| --- | --- | --- | --- |
| `can_interface` | string | `can0` | 준비된 SocketCAN 인터페이스 |
| `feedback_timeout_ms` | integer | `200` | 각 ID의 마지막 정상 해석 이후 허용 시간. 양수만 허용 |
| `enable_tx` | bool | `true` | CAN 송신 허용. false이면 상태 수신·발행만 수행 |
| `control_profile` | string | `hardware_drive` | `hardware_drive`, `vcan_reference` |
| `params_file` (launch 인자) | string | 설치된 `config/withus_driver.yaml` | 추가 노드 설정을 읽을 YAML 경로 |

노드 파라미터는 시작 시 적용하며 실행 중 변경할 수 없습니다. `feedback_timeout_ms`는 `0x10A/301/303/304` 각각에 적용됩니다. `base_frame_id`와 제어 제한값은 [기본 YAML](withus_bringup/config/withus_driver.yaml)을 참고하세요.

launch 대신 직접 실행할 수도 있습니다. 같은 인터페이스의 드라이버는 한 가지 방법으로 실행하세요.

```bash
ros2 run withus_driver withus_driver_node --ros-args \
  -p can_interface:=can0 -p feedback_timeout_ms:=200
```

### 조종기 A 입력 확인

드라이버에서 A 버튼을 조작한 전후에 `/withus/vehicle_status`를 조회합니다.

```bash
ros2 topic echo /withus/vehicle_status --once
```

`remote_a_raw`는 `0x10A`의 세 번째 바이트이며, `remote_a_selected`는 그 값이 `1`인지 나타냅니다. `status_10a_received=true`, `status_10a_valid=true`를 함께 확인하세요. A 선택 여부와 CAN 제어권 확보 여부는 별개입니다. 필드 의미와 미수신·만료 처리는 아래 [조종기 A 입력](#조종기-a-입력--0x10a)에 설명합니다.

### 조종기 A로 모드 선택하기

`hardware_drive`는 `0x501` 요청 1을 목표 20ms마다 계속 송신합니다. client가 별도 시작 서비스를 호출할 필요는 없습니다. A 선택·해제, 기어·속도·차량 모드, 피드백 만료나 고장 신호로 이 요청을 내리지 않습니다. 이 상태들은 구동 명령 수락·폐기와 진단에 별도로 사용합니다.

`DriverStatus.control_requested=true`는 요청 1의 송신 의도이며, 정상 요청 중 `state=MONITORING(1)`을 표시합니다. 실제 차량 모드는 `VehicleStatus.drive_mode`를 확인합니다. 모드 `3`은 반자율이며 모드 `1`과 구분합니다. 구동 상태는 `drive_*` 필드로 확인합니다.

소켓 송신 실패는 다음 주기에 재시도합니다. 수신 소켓 오류로 연결이 끊기면 송신을 멈추고, 인터페이스 복구 후 드라이버 재시작이 필요합니다. Ctrl+C 또는 선택적 `set_control(false)`는 구동 명령을 폐기하고 기본 1초 동안 요청 0으로 해제를 시도한 뒤 송신을 끝냅니다. 요청 해제 자체는 제동 명령이 아닙니다.

일반 A 모드 선택에는 서비스가 필요 없습니다. 서비스를 사용해 요청을 중지한 경우 `set_control(true)` 또는 노드 재시작으로 다시 허용하며, 이전 구동 목표는 복원하지 않습니다.

### client 주행 명령 연결하기

`hardware_drive`는 client의 `DriveCommand`를 받아 조향(`0x502`), 제동(`0x503`), 기어·속도(`0x504`)로 변환합니다. 일반 주행과 정지의 목표값은 client가 결정합니다. 드라이버는 임의의 정지용 제동량을 선택하지 않으며, `stop_brake_percent`는 이 프로필에서 사용하지 않습니다.

기존 드라이버를 종료하고 준비된 CAN 인터페이스에서 실행합니다.

```bash
source install/setup.bash
ros2 launch withus_bringup withus_bringup.launch.py
```

시작하면 `0x501` 요청 1을 20ms마다 계속 보냅니다. 새 client 명령을 받기 전에는 `0x502/503/504`를 보내지 않습니다. `BodyCommand`와 `0x506`은 이 프로필에서 지원하지 않습니다.

다음 순서로 현재 세션의 명령을 보냅니다. 이 순서는 드라이버의 명령 수락 규칙이며, 제조사가 검증한 실차 제어 확보 절차를 뜻하지 않습니다.

1. A 선택, 최신 `10A/301/303/304`, 차단 고장 없음, 유효한 기어·속도·조향 피드백과 모드 `1` 또는 `3`을 확인합니다. 새 세션은 정지 상태가 기본 200ms 동안 유지되어야 시작할 수 있습니다.
2. `DriverStatus.drive_ready`와 `session_id`를 읽고 **속도 0·현재 피드백과 같은 기어**인 첫 명령을 보냅니다. 조향각과 제동 비율도 client가 명시합니다.
3. `0x502/503/504` 전체 송신이 한 번 성공한 이후 새 `0x303`에서 모드 1과 최신 AD 신호 수신 상태를 확인하면 `drive_mode_confirmed=true`가 됩니다. 모드 3에서는 정지 상태의 속도 0 명령만 수락합니다. 이 조건을 확인하지 못하면 양의 속도 명령은 거부합니다.
4. 기어 변경은 정지 유지 후 속도 0 명령으로 별도로 요청합니다. `drive_gear_pending=false`와 실제 기어 일치를 확인한 뒤 **새 명령**으로 속도를 요청합니다. 모드·기어 확인 뒤 새 시각의 명령을 보내야 합니다. 확인 전에 받은 양의 속도를 저장해 나중에 실행하지 않습니다.
5. 주행 중에도 새 시각의 전체 명령을 주기적으로 갱신합니다. 일반 정지의 속도·제동 목표 역시 client가 지정합니다.

`drive_ready`는 현재 단계의 명령을 받을 수 있다는 뜻이며, 모든 속도·기어 요청을 허용한다는 뜻은 아닙니다. `drive_active`는 구동 프레임 valid=1을 송신하도록 설정한 상태입니다. 실제 응답은 `VehicleStatus`로 확인하며, 이 두 필드는 제어권 확보나 물리적 정지를 보장하지 않습니다.

명령의 `header.stamp`는 새 ROS 시각, `header.frame_id`는 설정한 기준 프레임(기본 `base_link`), `session_id`는 현재 드라이버 세션을 사용합니다. 기어는 0=P, 1=D, 2=N, 3=R이고 속도는 음수가 아닌 크기입니다. 후진은 R과 양의 속력으로 요청합니다. NaN·범위 초과, P/N에서 양의 속도, 양의 속도와 제동의 동시 요청, 중복·만료·미래 시각과 이전 세션은 거부합니다. 거부한 명령은 기존 명령의 만료 시간을 갱신하지 않습니다.

| 상황 | 구동 명령 처리 | `0x501` |
| --- | --- | --- |
| 유효한 새 명령 | 최신 목표로 교체하고 20ms 주기로 송신 | 요청 1 유지 |
| A 해제·미확정·만료, 차량 피드백 이상·고장, 확인한 AD 모드 상실 | 목표 폐기, 세션 교체, 모든 구동 valid=0을 정해진 시간 송신 | 요청 1 유지 |
| 명령 만료, 모드·기어 응답 시간 초과, 구동 송신 오류 | 같은 무효화 처리. 오래된 목표를 복구하지 않음 | 요청 1 유지 |
| 피드백 회복·A 재선택 | 정지 유지 후 새 세션의 속도 0 명령부터 대기 | 요청 1 유지 |
| 선택적인 `set_control(false)` 또는 Ctrl+C | 구동 명령 무효화 | 정해진 시간 요청 0 후 송신 중단 |
| 수신 소켓 연결 오류 | 송신 중단, 복구 후 노드 재시작 필요 | 송신 중단 |

무효화 구간은 기본 1초이며 반복 명령·오류로 연장하지 않습니다. 모든 구동 valid를 내린 뒤 구동 프레임 송신을 멈춥니다. **이 동작은 자동 제동이나 차량 정지 기능이 아닙니다.** 통신 단절 시에는 무효화 프레임 전달도 보장할 수 없습니다. client는 정상 정지 명령을 별도로 관리해야 하며, 실차의 명령 단절 대응은 차량의 실제 동작과 함께 검증해야 합니다.

| 설정 | 기본값 | 적용 |
| --- | --- | --- |
| `command_timeout_ms` | 200 | 수락한 명령의 최대 무갱신 시간; steady clock 기준 |
| `drive_invalidation_duration_ms` | 1000 | 구동 valid=0 송신 시간, 최대 5000 |
| `control_request_timeout_ms` | 1000 | 초기 모드 확인 및 각 기어 요청의 응답 제한 시간 |
| `stop_confirm_duration_ms` | 200 | 세션 시작·기어 변경 전 정지 유지 시간 |
| `stop_speed_threshold_mps` | 0.05 | 정지 판정의 실제 속도 절댓값 임계값 |
| `max_forward_speed_mps` / `max_reverse_speed_mps` | 1.0 / 0.5 | client 속도 요청 상한 |
| `max_steering_angle_rad` | 0.3 | client 조향 요청 절댓값 상한 |

`DriveCommand`의 제동은 0~100%입니다. CAN의 속도·조향·제동 분해능에 맞춰 기존 인코더가 반올림하며, 위 설정은 시작 시 YAML로 적용합니다. `set_control(true)`는 선택적으로 중지한 요청을 다시 허용하는 기능이며, 이전 목표를 복원하지 않습니다.

### 상태 모니터와 키보드 제어 함께 실행하기

터미널 1에서 기본 드라이버를 실행한 뒤, 터미널 2에는 상태 모니터를 실행합니다.

```bash
source install/setup.bash
ros2 launch withus_examples status_monitor.launch.py
```

터미널 3에는 키보드 client를 실행합니다. 이 터미널에 키를 입력하며 별도 GUI 창은 필요하지 않습니다.

```bash
source install/setup.bash
ros2 launch withus_examples keyboard_teleop.launch.py
```

| 키 | 동작 |
| --- | --- |
| `w` | 선택 속도로 직진, 조향 0. 양의 속도에서는 제동 0% |
| `a` / `d` | 선택 속도와 좌/우 최대 설정 조향각 요청. 양의 속도에서는 제동 0% |
| `s` | 속도 0, 제동 **80%**, 조향 **0** 요청 |
| `q` / `z` | 선택 속도 +0.1 / −0.1 m/s. 정지 중에는 주행을 시작하지 않음 |
| `1` / `2` / `3` / `4` | P / D / N / R 기어 요청. 정지 상태에서만 변경 |
| `Ctrl+C` | 명령 발행을 중단하고 client 종료 |

초기 선택 속도는 **0.1 m/s**입니다. 유효한 조작 키 입력이 **1초** 동안 없으면 **명령 발행을 중단**합니다. 이때 자동으로 80% 제동이나 조향 중립을 요청하지 않습니다. 주행 키를 누르고 있거나 반복 입력하여 조작을 갱신합니다. 전진·후진 속도 상한과 최대 조향각은 client 시작 시 실행 중인 드라이버에서 한 번 읽으며 기본값은 각각 1.0 m/s, 0.5 m/s, 0.3 rad입니다. 후진은 `4`로 R을 선택한 뒤 양의 속력으로 요청합니다.

client를 켜기만 해서는 구동 명령을 보내지 않습니다. 차량 정지와 A 선택 후 `s` 또는 기어 키로 준비를 시작합니다. client는 현재 기어·속도 0·제동 80%·조향 0을 먼저 보내고 새 AD 모드 피드백을 기다립니다. 기어 변경은 실제 정지 유지와 새 기어 피드백을 확인하며, 모드·기어 확인을 마친 뒤 **새 `w/a/d` 입력**으로 주행합니다. 주행 중 기어 키는 거부하므로 `s`로 정지를 요청한 뒤 실제 정지 상태에서 다시 선택합니다.

선택 속도가 0이면 `w/a/d`는 속도 0과 기존 제동 요청을 유지하며 조향을 바꿉니다. `q/z`로 속도를 0까지 낮추는 것 자체는 80% 제동 요청이 아니므로 제동은 `s`로 요청합니다.

기본 조향 부호는 `a=양수`, `d=음수`입니다. 제조사 문서에 좌우 부호가 명시되어 있지 않으므로 정지 상태에서 실제 방향을 확인하고, 반대라면 `left_steering_sign:=-1`로 실행합니다. `s`의 80%는 이 client의 정지 요청값이며 드라이버의 공통 정지 정책을 바꾸지 않습니다. 실제 제동·정지 응답은 상태 모니터에서 확인합니다.

입력 timeout이나 Ctrl+C로 발행을 중단하면 드라이버는 마지막 명령 수락 후 기본 200ms에 명령을 만료시키고, 기본 1초 동안 구동 valid=0을 보낸 뒤 구동 송신을 멈춥니다. 이는 실제 제동·정지 완료를 뜻하지 않습니다. 일반 정지가 필요하면 먼저 `s`를 입력하고 실제 응답을 확인하세요. A 해제, 상태 만료, 고장 또는 세션 변경 시에도 이전 목표를 폐기하며, 입력 timeout과 조건 복구 후에는 `s`나 기어 키로 다시 준비합니다. client 종료 후에도 드라이버의 `0x501` 연속 송신은 유지됩니다. 상세 실행 순서와 조정 파라미터는 [예제 매뉴얼](withus_examples/README.md)을 참고하세요.

### 단일 프레임 수신 확인

```bash
ros2 run withus_driver can_connection_check can0
```

최대 1초 동안 한 프레임을 기다려 ID·길이·payload를 출력하고 종료합니다. ROS 노드를 만들지 않는 일반 C++ 확인 프로그램입니다. 종료 코드는 성공 `0`, 연결·수신 실패 또는 타임아웃 `1`, 인자 개수 오류 `2`입니다.

### Python 상태 모니터 예제

드라이버 실행 후 다른 터미널에서 워크스페이스 환경을 적용하고 실행합니다.

```bash
source install/setup.bash
ros2 launch withus_examples status_monitor.launch.py
```

`/withus/status_monitor`가 상태 토픽을 구독하여 주행·운전·진단의 주요 값을 1초마다 출력합니다. ID별 미수신·유효성 만료를 구분하고, ROS 메시지 자체가 0.5초 넘게 끊기면 값 출력을 중단하고 경고합니다. 예제는 차량 명령을 보내지 않습니다.

출력 간격, 수신 timeout, 토픽 변경 방법은 [예제 사용 매뉴얼](withus_examples/README.md)을 참고하세요.

## ROS 토픽 인터페이스

방향은 드라이버 기준입니다. client는 드라이버가 발행하는 상태 토픽을 구독합니다.

| 방향 | 토픽 | 메시지 타입 | 주기 | 용도 |
| --- | --- | --- | --- | --- |
| 발행: 드라이버 → client | `/withus/vehicle_status` | `withus_msgs/msg/VehicleStatus` | 20ms 타이머, 목표 약 50Hz | 차량 피드백과 ID별 수신 유효성 |
| 발행 | `/withus/driver_status` | `withus_msgs/msg/DriverStatus` | 20ms | 제어 상태, 세션, 거부·송신 결과 |
| 발행 | `/withus/battery_state` | `sensor_msgs/msg/BatteryState` | 1초 타이머, 전압이 유효할 때 | 배터리 전압과 SOC |
| 발행 | `/withus/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 1초 | 연결·차량 피드백·제어 상태 진단 |
| 구독: client → 드라이버 | `/withus/drive_command` | `withus_msgs/msg/DriveCommand` | client의 갱신 주기 | 활성 세션의 기어·속도·조향·제동 목표 |
| 구독 | `/withus/body_command` | `withus_msgs/msg/BodyCommand` | client의 갱신 주기 | 활성 세션의 등화·경적 목표 |

`/withus/set_control` 서비스(`withus_msgs/srv/SetControl`)의 `enable`로 제어 시작·해제를 요청합니다. `hardware_drive`에서는 평소 호출하지 않아도 되며 자동 요청의 중지·재개 용도로 사용할 수 있습니다. 응답의 `accepted`는 요청 수락을 뜻하며, 전환 완료는 `/withus/driver_status`로 확인합니다. 이 표에서는 ROS 공통 로그·파라미터 인터페이스를 생략합니다.

주행 명령은 `vcan_reference`와 `hardware_drive`에서 허용하며, 장치 명령은 `vcan_reference`에서만 허용합니다. 메시지에는 `header`와 `session_id`가 필요합니다. 주행 명령의 `speed_mps`는 음수가 아닌 크기이며 방향은 `gear`로 선택합니다. 명령은 활성 세션·시각·범위 검증을 통과해야 합니다. 상세 필드는 [DriveCommand](withus_msgs/msg/DriveCommand.msg), [BodyCommand](withus_msgs/msg/BodyCommand.msg), [DriverStatus](withus_msgs/msg/DriverStatus.msg), [SetControl](withus_msgs/srv/SetControl.srv)을 참고하세요. 명령 구독 QoS는 Reliable, Volatile, KeepLast(1)입니다.

### DriverStatus의 제어 상태

| 필드 | 의미 |
| --- | --- |
| `control_profile` | 실행 중인 프로필 이름 |
| `control_requested` | 현재 `0x501` 요청 값 `1`의 송신 의도. 실제 전달·모드 전환 완료를 보장하지 않음 |
| `state` | `0` DISCONNECTED, `1` MONITORING, `2` REQUESTING, `3` ACTIVE, `4` STOPPING, `5` FAULT |
| `reason` | 현재 상태와 전환 이유 |
| `last_rejection` | 마지막 명령 거부 이유. 이후에도 마지막 기록을 유지 |
| `can_connected` | 수신 루프가 살아 있는지. 모든 피드백의 정상 수신을 보장하지 않음 |
| `enable_tx` | 시작 시 설정한 송신 허용 여부. 현재 송신 중이라는 뜻은 아님 |
| `feedback_valid` | 301/303/304 모두 최신인지. `hardware_drive`에서는 10A 최신성도 포함 |
| `session_id` | 현재 세션 식별자. `hardware_drive`에서는 구동 명령 세션 |
| `command_age_sec` | 마지막 수락한 주행 명령의 경과 시간. 미수신·폐기 후에는 NaN |
| `rejected_commands` | 거부한 주행·장치 명령의 누적 개수 |
| `tx_sent_frames` | 소켓 송신이 성공한 프레임 수. 차량 실행 완료를 의미하지 않음 |
| `tx_failed_frames` | 소켓 송신 실패 또는 송신 큐 사용 불가 횟수 |
| `drive_ready` | `hardware_drive`에서 현재 단계의 새 명령을 받을 수 있는지 |
| `drive_active` | 구동 valid=1 명령을 송신하도록 설정한 상태인지 |
| `drive_mode_confirmed` | 첫 구동 프레임 전체 송신 후 새 AD 모드·신호 수신 피드백을 확인했는지 |
| `drive_gear_pending` | 요청한 기어의 새 피드백을 기다리는지 |
| `drive_reason` | 구동 명령 준비·유지·폐기의 이유 |

`feedback_valid=true`는 차단 고장 없음이나 A 선택을 의미하지 않습니다. `state`는 `control_profile`과 함께 해석하세요. `vcan_reference`의 `ACTIVE`는 가상 제어 세션의 명령 수신 단계입니다.

`hardware_drive`의 `state`와 `reason`은 연속 `0x501` 요청의 상태를 나타내며 정상 송신 중 `MONITORING`입니다. 구동 상태는 `drive_*` 필드로 확인합니다. 명령 거부 이유는 `last_rejection`에 기록합니다. `vcan_reference`의 `drive_*`는 false·빈 문자열입니다.

```text
Withus 차량 → CAN 피드백 → withus_driver → /withus/vehicle_status → client
```

| 항목 | 설정 |
| --- | --- |
| 노드 | `/withus/withus_driver` |
| namespace / 상대 토픽 이름 | `withus` / `vehicle_status` |
| Publisher QoS | Reliable, Volatile, KeepLast(10) |
| 발행 주기 | 20ms 고정. 실제 실행 주기를 엄격히 보장하지는 않음 |

CAN 프레임이 도착할 때마다 별도 ROS 메시지를 보내는 방식이 아니라, 타이머가 네 ID의 최신 상태를 묶어 발행합니다. 따라서 각 필드의 데이터 수신 시각은 서로 다를 수 있습니다. CAN 피드백이 끊긴 동안에도 노드가 실행 중이면 마지막 값과 유효성 정보를 계속 발행합니다.

노드를 실행한 상태에서 인터페이스를 확인할 수 있습니다.

```bash
ros2 node info /withus/withus_driver
ros2 topic info /withus/vehicle_status --verbose
ros2 interface show withus_msgs/msg/VehicleStatus
ros2 topic echo /withus/vehicle_status --once
```

## VehicleStatus 메시지 필드

메시지 정의는 [VehicleStatus.msg](withus_msgs/msg/VehicleStatus.msg)에 있습니다. 모든 차량 상태 값은 차량이 보고한 피드백이며, 이 메시지를 발행한다고 차량에 명령이 전달되지는 않습니다.

### 필드 그룹과 유효성

| CAN ID | 필드 그룹 | 함께 확인할 유효성 |
| --- | --- | --- |
| `0x10A` | 조종기 A 입력의 원본 값과 선택 여부 | `status_10a_valid` |
| `0x301` | 배터리 전압, 비상정지·고장·범퍼·등화 등 진단 및 장치 상태 | `status_301_valid` |
| `0x303` | 기어, 운전 모드, 브레이크 유지, 차량이 보고한 요청 속도, 배터리 SOC | `status_303_valid` |
| `0x304` | 실제 속도, 조향각, 제동압 | `status_304_valid` |

차량 피드백 세 프레임의 28개 신호와 조종기 A 입력을 제공합니다. `0x10A`에는 메시지 카운터가 정의되어 있지 않으므로 카운터를 발행하거나 불연속을 계산하지 않습니다. 데이터시트의 다른 버스·장치 상세 프레임은 포함하지 않습니다. 차량 신호 정의는 [CAN 데이터시트](docs/withus_CAN_datasheet.pdf) PDF 9, 11, 13쪽, A 입력은 14쪽을 기준으로 합니다.

아래 표의 bool 의미는 **해당 그룹의 valid가 true일 때** 해석하세요. 미수신 또는 오래된 데이터에서는 false가 정상이나 꺼짐을 보장하지 않습니다.

### 헤더

`header`의 타입은 `std_msgs/Header`입니다.

| 필드 | 타입 | 의미 |
| --- | --- | --- |
| `header.stamp` | `builtin_interfaces/Time` | 드라이버가 상태 메시지를 구성한 ROS 시각. CAN 측정 시각이나 하드웨어 수신 시각이 아님 |
| `header.frame_id` | `string` | 차량 기준 프레임 이름. `base_frame_id` 파라미터, 기본 `base_link` |

`header.stamp`가 갱신돼도 일부 CAN 데이터는 오래된 값일 수 있습니다. 각 그룹의 `status_<id>_valid`, `status_<id>_stamp`, `status_<id>_age_sec`를 함께 확인하세요.

### 조종기 A 입력 — 0x10A

| 필드 | 타입 | 의미 | 첫 정상 수신 전 |
| --- | --- | --- | --- |
| `remote_a_raw` | `uint8` | 세 번째 데이터 바이트 `data[2]`의 원본 값, 0~255 보존 | `255` |
| `remote_a_selected` | `bool` | 마지막으로 받은 원본 값이 `1`인지 | false |

확인된 차량에서는 A 선택 전 `0`, 선택 후 `1`이 관측되었습니다. 다른 원본 값은 그대로 보관하며 `remote_a_selected=false`로 표시합니다. 알 수 없는 값이 왔을 때 이를 확인된 선택 해제로 해석하지 마세요. `255`도 수신 가능한 원본 값이므로 미수신 여부는 `status_10a_received`로 구분합니다.

`status_10a_valid`는 프레임 형식과 최신성의 판정이며, 원본 값의 의미가 확인되었거나 제어권을 확보했다는 뜻은 아닙니다. 수신이 만료되면 마지막 `remote_a_raw`와 `remote_a_selected`는 유지됩니다. 따라서 `remote_a_selected=true`만 사용하지 말고 `status_10a_valid`도 확인해야 합니다. `hardware_drive`는 A 입력으로 구동 명령을 수락·폐기하며 이 값으로 `0x501` 요청 송신을 차단하지 않습니다. `vcan_reference`는 A 입력을 시작 조건으로 사용하지 않습니다.

### 주행 상태 — 0x304

| 필드 | 타입 | 단위 | 의미 |
| --- | --- | --- | --- |
| `speed_mps` | `float64` | m/s | 차량이 보고한 실제 속도 |
| `steering_angle_rad` | `float64` | rad | 차량이 보고한 조향각. CAN 데이터시트의 부호와 기준을 따름 |
| `brake_pressure_pa` | `float64` | Pa | 제동압. 예: `1000000.0`은 1MPa |

세 필드 모두 `status_304_valid`를 기준으로 사용합니다. 조향각을 Ackermann 조향으로 사용할 때는 차량의 조향 기준과 좌우 부호의 대응을 별도로 확인하세요.

### 운전 상태 — 0x303

| 필드 | 타입 | 단위 / 값 | 의미 |
| --- | --- | --- | --- |
| `gear` | `uint8` | 아래 기어 상수 | 차량이 보고한 기어 상태 |
| `drive_mode` | `uint8` | 아래 운전 모드 상수 | 차량이 보고한 운전 모드 |
| `brake_hold_active` | `bool` | true / false | true이면 브레이크 유지 상태 |
| `requested_speed_mps` | `float64` | m/s | 차량이 보고한 요청 속도. 실제 속도 `speed_mps`와 구분 |
| `battery_soc` | `float64` | 0.0~1.0 | 배터리 충전 비율. `0.75`는 75% |

이 그룹은 `status_303_valid`를 기준으로 사용합니다. `requested_speed_mps`는 상태 피드백이며 이 토픽으로 차량에 입력하는 목표 속도가 아닙니다.

기어 상수:

| 값 | 메시지 상수 | 의미 |
| --- | --- | --- |
| `0` | `GEAR_P` | P, 주차 |
| `1` | `GEAR_D` | D, 전진 |
| `2` | `GEAR_N` | N, 중립 |
| `3` | `GEAR_R` | R, 후진 |
| `255` | `GEAR_UNKNOWN` | 정상 해석한 기어 정보가 없음 |

운전 모드 상수:

| 값 | 메시지 상수 | 의미 |
| --- | --- | --- |
| `0` | `DRIVE_MODE_REMOTE` | Remote, 원격 제어 모드 |
| `1` | `DRIVE_MODE_AD` | AD, 자율주행 모드 |
| `2` | `DRIVE_MODE_PARALLEL` | Parallel, 병렬 운전 모드 |
| `3` | `DRIVE_MODE_SEMI_AUTONOMOUS` | Semi-auto, 반자율 모드 |
| `4~15` | 이름이 정의된 상수 없음 | CAN의 원시 모드 값을 그대로 전달 |
| `255` | `DRIVE_MODE_UNKNOWN` | 정상 해석한 운전 모드 정보가 없음 |

기어·모드를 한 번 받은 후 수신이 끊기면 마지막 값을 유지합니다. UNKNOWN만 확인하지 말고 `status_303_valid`도 확인하세요.

### 진단·장치 상태 — 0x301

이 그룹은 모두 `status_301_valid`를 기준으로 사용합니다. bool 설명은 true일 때의 의미입니다.

| 필드 | 타입 | 단위 / 의미 |
| --- | --- | --- |
| `battery_voltage_v` | `float64` | 배터리 전압, V |
| `emergency_stop_pressed` | `bool` | 물리 비상정지 입력이 눌린 상태 |
| `motor_fault` | `bool` | 모터 고장 신호 |
| `remote_fault` | `bool` | 리모컨 고장 신호 |
| `ad_signal_received` | `bool` | 차량이 AD 신호를 수신한 상태 |
| `dbs_fault` | `bool` | DBS 제동 계통 고장 신호 |
| `parallel_driving_active` | `bool` | 병렬 운전 활성 상태 |
| `bms_fault` | `bool` | BMS 배터리 관리 계통 고장 신호 |
| `eps_fault` | `bool` | EPS 조향 계통 고장 신호 |
| `front_bumper_triggered` | `bool` | 앞 범퍼 감지 신호 |
| `rear_bumper_triggered` | `bool` | 뒤 범퍼 감지 신호 |
| `headlight_on` | `bool` | 전조등 켜짐 |
| `left_indicator_on` | `bool` | 좌측 방향등 켜짐 |
| `brake_light_on` | `bool` | 제동등 켜짐 |
| `oil_level_abnormal` | `bool` | 오일 레벨 이상 신호 |
| `horn_on` | `bool` | 경적 동작 상태 |
| `right_indicator_on` | `bool` | 우측 방향등 켜짐 |

`ad_signal_received`와 `parallel_driving_active`는 CAN 원시 비트의 active-low 의미를 드라이버에서 반영한 값입니다. ROS 소비자가 다시 반전할 필요는 없습니다. `ad_signal_received=true`만으로 client가 차량 제어 권한을 확보했다고 판단하지 마세요.

### ID별 수신 메타데이터

아래 `<id>`는 `10a`, `301`, `303`, `304` 중 하나입니다. 각 그룹은 독립적으로 관리됩니다. 카운터 관련 두 필드는 `301/303/304`에만 있으며, `status_10a_counter`나 `status_10a_counter_discontinuities`는 없습니다.

| 필드 패턴 | 타입 | 의미 | 첫 정상 수신 전 |
| --- | --- | --- | --- |
| `status_<id>_received` | `bool` | 드라이버 시작 이후 해당 ID를 한 번 이상 정상 해석했는지 | false |
| `status_<id>_valid` | `bool` | 정상 해석한 데이터가 있고, 나이가 0 이상이며 `feedback_timeout_ms` 이하인지 | false |
| `status_<id>_stamp` | `builtin_interfaces/Time` | 마지막 정상 해석 프레임을 소켓에서 읽은 직후의 호스트 ROS 시각 | sec=0, nanosec=0 |
| `status_<id>_age_sec` | `float64` | 마지막 정상 해석 이후 경과 시간, 초. steady clock으로 계산 | NaN |
| `status_<id>_counter` | `uint8` | 마지막 정상 해석 프레임의 4비트 카운터, 0~15 | `255` (`COUNTER_UNKNOWN`) |
| `status_<id>_counter_discontinuities` | `uint64` | 카운터가 직전의 다음 값과 달랐던 누적 횟수. 반복값도 포함 | 0 |
| `status_<id>_rejected_frames` | `uint64` | 해당 ID로 분기했으나 형식·값 검증에 실패한 누적 프레임 수 | 0 |

`received=true`는 수신 이력이 있다는 뜻입니다. 수신이 끊겨도 마지막 값과 received는 유지되고 timeout을 넘으면 valid가 false가 됩니다. 형식이나 값이 잘못된 프레임은 기존 값과 정상 수신 시각을 갱신하지 않습니다.

카운터는 `15 → 0` 순환을 정상 처리합니다. 불연속 횟수는 정확한 유실 프레임 수가 아니며 valid 판정에도 사용하지 않습니다. 고정 payload를 반복 보내는 시험에서는 카운터가 반복되므로 이 값이 증가할 수 있습니다. 두 누적 횟수는 드라이버 재시작 시 0으로 초기화됩니다.

`age_sec`는 커널 수신 큐 체류 시간이나 ROS 전달·화면 표시 지연까지 측정하지 않습니다. 각 ID의 stamp는 각각의 수신 시각이며 동시에 측정된 데이터라는 의미는 아닙니다.

### 속도 유효성 필드

아래 필드는 `0x304`의 메타데이터와 같은 값을 제공합니다.

| 필드 | 타입 | 동일한 값을 갖는 필드 |
| --- | --- | --- |
| `speed_received` | `bool` | `status_304_received` |
| `speed_valid` | `bool` | `status_304_valid` |
| `speed_age_sec` | `float64` | `status_304_age_sec` |

### 값을 사용하는 기준

| 수신 상태 | received | valid | 값의 의미 |
| --- | --- | --- | --- |
| 첫 정상 수신 전 | false | false | 연속량은 NaN, 기어·모드는 255, bool은 false. 실제 차량 상태로 사용하지 않음 |
| 정상 해석된 최신 데이터 | true | true | 해당 그룹의 값과 고장 신호를 읽을 수 있음 |
| timeout 초과 | true | false | 마지막 정상 수신 값을 보관 중. 최신 상태로 사용하지 않음 |

예를 들어 SOC는 `status_303_valid`를 확인한 뒤 `battery_soc * 100`으로 표시하고, 전압은 별도로 `status_301_valid`를 확인합니다. 속도는 `speed_valid` 또는 `status_304_valid`를 확인한 뒤 사용합니다.

`valid=true`는 데이터의 신선도를 뜻하며 차량에 고장이 없다는 뜻은 아닙니다. 또한 소비자는 ROS 메시지 자체의 수신 중단도 감지해야 합니다. 드라이버가 종료되면 마지막으로 받은 메시지의 valid가 자동으로 false로 바뀌지는 않습니다.

## 동작과 통신 복구

```text
CAN → SocketCanTransport → ID별 디코더 → VehicleState → VehicleStatus 토픽
             전용 수신 스레드              ↕ mutex        ROS 타이머
```

`SingleThreadedExecutor`가 ROS 콜백을 처리하고 별도 `std::thread`가 CAN을 수신합니다. 발행 콜백은 잠금 안에서 상태를 복사한 뒤 잠금을 풀고 메시지를 구성합니다. 디코더는 표준 데이터 프레임, 길이 8바이트와 정의된 물리값 범위를 검사합니다. `0x10A` A 입력은 의미가 정의되지 않은 원본 값도 보존합니다.

- `receive(frame, 100)`의 100ms는 입력이 없을 때의 대기 시간입니다. 프레임마다 100ms를 기다리는 구조가 아닙니다.
- 특정 ID만 오지 않으면 해당 ID의 유효성만 만료됩니다. 소켓이 유지된 상태에서 정상 프레임이 다시 오면 자동으로 갱신됩니다.
- 소켓 오류가 발생하면 수신 루프가 종료되고 상태 발행은 계속됩니다. 마지막 값의 경과 시간이 늘어나며 유효성이 만료됩니다. 수신 오류로 루프가 종료된 경우 인터페이스를 복구하고 노드를 재시작하세요.
- `Ctrl+C`로 종료하면 해당 프로필의 해제 절차를 제한 시간 동안 시도한 뒤 타이머 중단 → 수신 종료 요청 → 스레드 `join()` → 소켓 닫기 순서로 정리합니다. `hardware_drive`는 구동 명령을 폐기하고 `0x502/503/504`를 무효화하며 자동 제동하지 않습니다. `vcan_reference`는 가상 차량의 정지·해제를 수행합니다.
- 소비자는 ROS 상태 메시지 자체가 끊기는 상황도 감지해야 합니다. 드라이버가 종료되면 마지막으로 받은 `valid=true`가 자동 변경되지는 않습니다.

## 소프트웨어 테스트

CAN 연결 없이 상태 전환 로직을 검사합니다.

```bash
colcon test --packages-select withus_driver
colcon test-result --test-result-base build/withus_driver --verbose
```

ROS/CAN 통합 테스트는 이미 활성화된 `vcan0`에서 실행합니다. 같은 가상 CAN을 사용하는 다른 송신 프로그램과 드라이버를 종료한 뒤 실행하세요.

```bash
source install/setup.bash
python3 src/withus_driver_ros2/withus_driver/test/test_hardware_drive.py
python3 src/withus_driver_ros2/withus_driver/test/test_bringup_examples.py
```

테스트는 커널 링크 종류가 `vcan`인지 검사하며 물리 CAN을 거부합니다. 가상 피드백으로 모드 요청·해제·Fault와 송신 ID를 확인하며, 실제 차량의 제동이나 정지 성능을 검증하지는 않습니다.

## 관련 문서

- [VehicleStatus 메시지 정의](withus_msgs/msg/VehicleStatus.msg)
- [CAN 데이터시트](docs/withus_CAN_datasheet.pdf)
- [워크스페이스 문서 모음](../withus_driver-docs/README.md)

워크스페이스 문서 링크는 형제 디렉터리 `withus_driver-docs`를 기준으로 합니다. 이 디렉터리는 소스 Git 저장소에 포함되지 않습니다.

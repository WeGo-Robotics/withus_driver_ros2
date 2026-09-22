# Withus ROS 2 Driver

Withus 플랫폼의 CAN 피드백을 수신하고 ROS 2 토픽으로 제공하는 C++17 드라이버입니다.
Linux SocketCAN으로 `0x301`, `0x303`, `0x304`의 28개 신호를 해석하며, ID별 수신 여부와 데이터 유효성을 함께 발행합니다.

이 패키지는 차량 상태를 읽는 수신 전용 드라이버입니다.

## 패키지 구성

| 패키지 | 역할 |
| --- | --- |
| [withus_driver](withus_driver/) | SocketCAN 통신, 프레임별 디코딩, ID별 상태 저장, ROS 상태 발행 |
| [withus_msgs](withus_msgs/) | [VehicleStatus.msg](withus_msgs/msg/VehicleStatus.msg) 및 메시지 생성 설정 |
| [withus_bringup](withus_bringup/) | 드라이버 launch, CAN 설정 스크립트, 실행 의존성 |
| [withus_examples](withus_examples/README.md) | Python 상태 구독 예제, 유효성 표시와 ROS 메시지 수신 중단 감시 |

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
│   │   └── withus_driver_node.hpp
│   ├── src/
│   │   ├── socket_can_transport.cpp
│   │   ├── can_connection_check.cpp
│   │   ├── withus_protocol.cpp
│   │   ├── withus_driver_node.cpp
│   │   └── main.cpp
│   ├── CMakeLists.txt
│   └── package.xml
├── withus_msgs/
│   ├── msg/VehicleStatus.msg
│   ├── CMakeLists.txt
│   └── package.xml
├── withus_bringup/
│   ├── launch/withus_bringup.launch.py
│   ├── scripts/setup_can.sh
│   ├── CMakeLists.txt
│   └── package.xml
└── withus_examples/
    ├── withus_examples/status_monitor.py
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
ros2 launch withus_bringup withus_bringup.launch.py \
  can_interface:=can0 feedback_timeout_ms:=200
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

두 파라미터는 시작 시 적용하며 실행 중 변경할 수 없습니다. `feedback_timeout_ms`는 세 ID에 각각 적용됩니다.

launch 대신 직접 실행할 수도 있습니다. 같은 인터페이스의 드라이버는 한 가지 방법으로 실행하세요.

```bash
ros2 run withus_driver withus_driver_node --ros-args \
  -p can_interface:=can0 -p feedback_timeout_ms:=200
```

### 단일 프레임 수신 확인

```bash
ros2 run withus_driver can_connection_check can0
```

최대 1초 동안 한 프레임을 기다려 ID·길이·payload를 출력하고 종료합니다. ROS 노드를 만들지 않는 일반 C++ 확인 프로그램입니다. 종료 코드는 성공 `0`, 연결·수신 실패 또는 타임아웃 `1`, 인자 개수 오류 `2`입니다.

### Python 상태 모니터 예제

드라이버 실행 후 다른 터미널에서 워크스페이스 환경을 적용하고 실행합니다.

```bash
source install/setup.bash
ros2 run withus_examples status_monitor
```

`/withus/status_monitor`가 상태 토픽을 구독하여 주행·운전·진단의 주요 값을 1초마다 출력합니다. ID별 미수신·유효성 만료를 구분하고, ROS 메시지 자체가 0.5초 넘게 끊기면 값 출력을 중단하고 경고합니다. 예제는 차량 명령을 보내지 않습니다.

출력 간격, 수신 timeout, 토픽 변경 방법은 [예제 사용 매뉴얼](withus_examples/README.md)을 참고하세요.

## ROS 토픽 인터페이스

방향은 드라이버 기준입니다. 고객 프로그램이나 대시보드는 드라이버가 발행하는 상태 토픽을 구독합니다.

| 방향 | 토픽 | 메시지 타입 | 주기 | 용도 |
| --- | --- | --- | --- | --- |
| 발행: 드라이버 → 고객 프로그램 | `/withus/vehicle_status` | `withus_msgs/msg/VehicleStatus` | 20ms 타이머, 목표 약 50Hz | 차량 피드백과 ID별 수신 유효성 |
| 구독: 고객 프로그램 → 드라이버 | 없음 | — | — | 차량 명령을 받는 입력 토픽 없음 |

이 표는 차량 기능용 인터페이스를 설명합니다. ROS 공통 로그·파라미터 인터페이스는 생략합니다. 차량 조작용 서비스도 제공하지 않습니다.

```text
Withus 차량 → CAN 피드백 → withus_driver → /withus/vehicle_status → 고객 프로그램
```

| 항목 | 설정 |
| --- | --- |
| 노드 | `/withus/withus_driver` |
| namespace / 상대 토픽 이름 | `withus` / `vehicle_status` |
| Publisher QoS | Reliable, Volatile, KeepLast(10) |
| 발행 주기 | 20ms 고정. 실제 실행 주기를 엄격히 보장하지는 않음 |

CAN 프레임이 도착할 때마다 별도 ROS 메시지를 보내는 방식이 아니라, 타이머가 세 ID의 최신 상태를 묶어 발행합니다. 따라서 각 필드의 데이터 수신 시각은 서로 다를 수 있습니다. CAN 피드백이 끊긴 동안에도 노드가 실행 중이면 마지막 값과 유효성 정보를 계속 발행합니다.

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
| `0x301` | 배터리 전압, 비상정지·고장·범퍼·등화 등 진단 및 장치 상태 | `status_301_valid` |
| `0x303` | 기어, 운전 모드, 브레이크 유지, 차량이 보고한 요청 속도, 배터리 SOC | `status_303_valid` |
| `0x304` | 실제 속도, 조향각, 제동압 | `status_304_valid` |

각 ID의 메시지 카운터를 포함해 세 프레임의 28개 신호를 제공합니다. 데이터시트의 다른 버스·장치 상세 프레임은 포함하지 않습니다. 신호 정의는 [CAN 데이터시트](docs/withus_CAN_datasheet.pdf) PDF 9, 11, 13쪽을 기준으로 합니다.

아래 표의 bool 의미는 **해당 그룹의 valid가 true일 때** 해석하세요. 미수신 또는 오래된 데이터에서는 false가 정상이나 꺼짐을 보장하지 않습니다.

### 헤더

`header`의 타입은 `std_msgs/Header`입니다.

| 필드 | 타입 | 의미 |
| --- | --- | --- |
| `header.stamp` | `builtin_interfaces/Time` | 드라이버가 상태 메시지를 구성한 ROS 시각. CAN 측정 시각이나 하드웨어 수신 시각이 아님 |
| `header.frame_id` | `string` | 차량 기준 프레임 이름. `base_link` 고정 |

`header.stamp`가 갱신돼도 일부 CAN 데이터는 오래된 값일 수 있습니다. 각 그룹의 `status_<id>_valid`, `status_<id>_stamp`, `status_<id>_age_sec`를 함께 확인하세요.

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

`ad_signal_received`와 `parallel_driving_active`는 CAN 원시 비트의 active-low 의미를 드라이버에서 반영한 값입니다. ROS 소비자가 다시 반전할 필요는 없습니다. `ad_signal_received=true`만으로 고객 프로그램이 차량 제어 권한을 확보했다고 판단하지 마세요.

### ID별 수신 메타데이터

아래 `<id>`는 `301`, `303`, `304` 중 하나입니다. 예를 들어 `status_<id>_valid`는 실제 메시지의 `status_301_valid`, `status_303_valid`, `status_304_valid` 세 필드를 뜻합니다. 각 그룹은 독립적으로 관리됩니다.

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

`age_sec`는 커널 수신 큐 체류 시간이나 ROS 전달·화면 표시 지연까지 측정하지 않습니다. 세 ID의 stamp는 각각의 수신 시각이며 동시에 측정된 데이터라는 의미는 아닙니다.

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

`rclcpp::spin()`이 ROS 콜백을 처리하고 별도 `std::thread`가 CAN을 수신합니다. 발행 콜백은 잠금 안에서 상태를 복사한 뒤 잠금을 풀고 메시지를 구성합니다. 디코더는 표준 데이터 프레임, 길이 8바이트, 필드 범위를 검사합니다.

- `receive(frame, 100)`의 100ms는 입력이 없을 때의 대기 시간입니다. 프레임마다 100ms를 기다리는 구조가 아닙니다.
- 특정 ID만 오지 않으면 해당 ID의 유효성만 만료됩니다. 소켓이 유지된 상태에서 정상 프레임이 다시 오면 자동으로 갱신됩니다.
- 소켓 오류가 발생하면 수신 루프가 종료되고 상태 발행은 계속됩니다. 마지막 값의 경과 시간이 늘어나며 유효성이 만료됩니다. 수신 오류로 루프가 종료된 경우 인터페이스를 복구하고 노드를 재시작하세요.
- `Ctrl+C`로 종료하면 발행 타이머 중단 → 수신 종료 요청 → 스레드 `join()` → 소켓 닫기 순서로 정리합니다.
- 소비자는 ROS 상태 메시지 자체가 끊기는 상황도 감지해야 합니다. 드라이버가 종료되면 마지막으로 받은 `valid=true`가 자동 변경되지는 않습니다.

## 관련 문서

- [VehicleStatus 메시지 정의](withus_msgs/msg/VehicleStatus.msg)
- [CAN 데이터시트](docs/withus_CAN_datasheet.pdf)
- [워크스페이스 문서 모음](../withus_driver-docs/README.md)

워크스페이스 문서 링크는 형제 디렉터리 `withus_driver-docs`를 기준으로 합니다. 이 디렉터리는 소스 Git 저장소에 포함되지 않습니다.

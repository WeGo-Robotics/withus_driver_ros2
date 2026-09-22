# Withus Python 예제

`withus_examples`는 공개 ROS 인터페이스로 차량 상태를 읽는 Python 예제 패키지입니다. `status_monitor`는 `withus_msgs/msg/VehicleStatus`를 구독하여 주요 값을 요약하고 수신 중단을 감지합니다.

## 빌드

워크스페이스 루트에서 메시지 패키지와 예제를 빌드합니다.

```bash
cd ~/ros2_workspace/withus_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to withus_examples
source install/setup.bash
```

실차에서 사용하려면 별도로 빌드한 `withus_bringup`과 준비된 CAN 인터페이스가 필요합니다. 예제 자체는 `rclpy`, `rcl_interfaces`, `withus_msgs`를 사용하며 CAN 소켓을 열지 않습니다.

## 실행

드라이버를 실행한 터미널:

```bash
ros2 launch withus_bringup withus_bringup.launch.py can_interface:=can0
```

다른 터미널에서 예제를 실행합니다.

```bash
cd ~/ros2_workspace/withus_ws
source install/setup.bash
ros2 run withus_examples status_monitor
```

예제 노드는 `/withus/status_monitor`, 구독 토픽은 `/withus/vehicle_status`입니다. 예제는 차량 명령을 보내지 않으며, Ctrl+C로 종료해도 드라이버는 계속 실행됩니다.

다른 토픽으로 연결할 때는 상대 이름 `vehicle_status`를 remap합니다.

```bash
ros2 run withus_examples status_monitor --ros-args \
  -r vehicle_status:=/another_robot/vehicle_status
```

## 표시 내용

| 그룹 | 주요 표시 값 | 유효성 기준 |
| --- | --- | --- |
| `301` | 전압, 비상정지 입력, 모터·리모컨·DBS·BMS·EPS 고장 목록 | `status_301_received`, `status_301_valid` |
| `303` | 기어, 운전 모드, 브레이크 유지, 요청 속도, SOC | `status_303_received`, `status_303_valid` |
| `304` | 실제 속도, 조향각, 제동압 | `status_304_received`, `status_304_valid` |

유효한 데이터를 수신했을 때의 출력 예시입니다.

```text
ROS status reception active.
301: VALID (age at publish=0.010 s) | voltage=48.0 V, e_stop=False, faults=none
303: VALID (age at publish=0.010 s) | gear=P, mode=REMOTE, hold=False, requested_speed=0.00 m/s, SOC=75.0%
304: VALID (age at publish=0.010 s) | speed=0.00 m/s, steering=0.000 rad, brake=0 Pa
```

속도는 m/s, 조향각은 rad, 제동압은 Pa, 전압은 V입니다. SOC는 메시지의 0~1 비율을 퍼센트로 변환합니다. `requested_speed`는 차량이 보고한 요청값이고 `speed`는 실제 속도입니다. 운전 모드나 기어의 알려지지 않은 값은 `UNKNOWN(값)`으로 표시합니다.

`faults=none`은 표시 대상인 다섯 고장 신호가 false라는 의미입니다. 다른 고장·범퍼·등화 상태까지 모두 정상임을 뜻하지 않습니다. 전체 필드는 `ros2 interface show withus_msgs/msg/VehicleStatus`로 확인할 수 있습니다.

## 유효성과 수신 중단

| 출력 | 의미 |
| --- | --- |
| `Waiting for vehicle_status.` | ROS 메시지를 한 번도 받지 못함 |
| `NOT_RECEIVED` | ROS 메시지는 있지만 해당 CAN ID의 정상 해석 이력이 없음 |
| `VALID` | 해당 ID의 데이터가 드라이버 기준으로 유효함. 차량 고장 유무와는 별개 |
| `STALE` | 해당 ID의 수신 유효성이 만료됨. 오래된 물리량·bool 값을 정상값처럼 출력하지 않음 |
| `No ROS status messages...` | 토픽 수신이 제한 시간 동안 중단됨. 캐시된 valid가 true여도 상태값 출력을 중단 |
| `ROS status reception active.` | ROS 메시지 수신 시작 또는 재개 |

대기·토픽 중단·재개 로그는 연결 상태가 바뀔 때 출력합니다. 토픽이 수신되는 동안에는 각 ID의 상태를 설정한 주기로 요약합니다.

`age at publish`는 드라이버가 메시지에 넣은 데이터 나이입니다. ROS 전송 지연이나 예제의 출력 대기 시간을 포함한 전체 지연값이 아닙니다. 예제의 토픽 수신 감시는 별도로 로컬 `time.monotonic()`을 사용합니다.

## 시작 파라미터

| 이름 | 타입 | 기본값 | 의미 |
| --- | --- | --- | --- |
| `report_period_sec` | double | `1.0` | 상태 요약 출력 간격, 초 |
| `status_topic_timeout_sec` | double | `0.5` | 마지막 ROS 메시지 수신 이후 허용 시간, 초 |

두 값은 유한한 0.1초 이상 값이어야 하며 시작 시에만 설정합니다. ROS 파라미터 타입에 맞게 `1.0`처럼 실수로 입력합니다.

```bash
ros2 run withus_examples status_monitor --ros-args \
  -p report_period_sec:=2.0 \
  -p status_topic_timeout_sec:=1.0
```

수신 상태는 steady-clock 타이머로 0.1초마다 확인하므로, 설정값을 넘긴 다음 타이머 실행 시 중단을 표시합니다. 실제 실행 지연에 따라 확인 시점이 늦어질 수 있습니다. 이 timeout은 드라이버의 ID별 CAN 피드백 timeout인 `feedback_timeout_ms`와 별도입니다.

## 코드 구성

- `on_status()`: 최신 메시지와 로컬 수신 시각만 저장합니다. 구독 QoS는 Reliable / Volatile / KeepLast(1)입니다.
- `on_timer()`: ROS 메시지 중단을 판단하고, 연결된 동안 상태 요약을 출력합니다. ROS 시뮬레이션 시간이 멈춰도 감시 타이머는 동작합니다.
- `describe_group()`: ID별 received/valid를 확인한 뒤 상태값을 표시합니다.
- `main()`: 단일 스레드 executor로 콜백을 실행하고 종료 시 노드를 정리합니다.

콜백에서 매 프레임 로그를 출력하지 않고 최신 메시지만 저장하므로 수신 빈도와 출력 빈도를 분리합니다. Python 예제에 CAN 디코딩이나 드라이버 내부 라이브러리 의존성은 없습니다.

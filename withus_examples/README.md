# Withus Python 예제

`withus_examples`는 드라이버의 공개 ROS 인터페이스를 사용하는 client 패키지입니다. 드라이버와 client를 각각 다른 터미널에서 실행합니다. 각 예제 launch는 해당 client 하나만 시작하며, 드라이버 실행·CAN 설정·제어 서비스 호출을 수행하지 않습니다.

| 예제 launch | 기능 |
| --- | --- |
| `status_monitor.launch.py` | 상태 요약과 수신 중단 감시 |
| `brake_response_check.launch.py` | 정지·중립 상태에서 명시한 제동 명령의 압력 응답 관측 |

## 빌드

워크스페이스 루트에서 드라이버와 예제를 함께 빌드합니다.

```bash
cd ~/ros2_workspace/withus_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to withus_bringup withus_examples
source install/setup.bash
```

## 드라이버 실행: 터미널 1

CAN 인터페이스를 준비한 뒤 기본 launch를 실행합니다. 기본 프로필은 `hardware_drive`이며 CAN 송신이 활성화됩니다.

```bash
source install/setup.bash
ros2 launch withus_bringup withus_bringup.launch.py can_interface:=can0
```

드라이버는 `0x501 request=1`을 계속 보내며 조종기 A로 모드를 선택합니다. client의 유효한 `DriveCommand`가 없으면 구동 명령 `0x502/503/504`를 시작하지 않습니다.

## 상태 모니터: 터미널 2

```bash
source install/setup.bash
ros2 launch withus_examples status_monitor.launch.py
```

예제 노드는 `/withus/status_monitor`, 구독 토픽은 `/withus/vehicle_status`입니다. 상태 모니터는 차량 명령을 보내지 않습니다. Ctrl+C로 client를 종료해도 터미널 1의 드라이버는 계속 실행됩니다.

설정을 바꾸려면 launch 인자를 사용합니다.

```bash
ros2 launch withus_examples status_monitor.launch.py \
  namespace:=withus \
  report_period_sec:=2.0 \
  status_topic_timeout_sec:=1.0
```

다른 토픽으로 연결할 때는 직접 실행하여 상대 이름 `vehicle_status`를 remap할 수도 있습니다.

```bash
ros2 run withus_examples status_monitor --ros-args \
  -r vehicle_status:=/another_robot/vehicle_status
```

## 정지 상태 제동 응답: 터미널 2

터미널 1의 동일한 `withus_bringup.launch.py`를 사용합니다. 차량을 정지·중립(N)으로 유지하고 A를 선택한 뒤, 요청 비율과 시간을 명시합니다. 시험 중 제동력이 달라질 수 있으므로 차량이 굴러가지 않게 유지하세요.

```bash
source install/setup.bash
ros2 launch withus_examples brake_response_check.launch.py \
  brake_percent:=10 duration:=1.0
```

`10`은 명령 사용 예시이며 검증된 정지 비율이 아닙니다.

| launch 인자 | 기본값 | 의미 |
| --- | --- | --- |
| `brake_percent` | 없음, 필수 | client가 선택한 제동 비율, 유한한 0~100% |
| `duration` | `1.0` | 명령 발행 시간, 0 초과 1.5 이하, 초 |
| `namespace` | `withus` | 드라이버 토픽과 연결할 namespace |

기존 직접 실행 방식도 지원합니다.

```bash
ros2 run withus_examples brake_response_check --brake-percent 10 --duration 1.0
```

이 예제는 `/withus/drive_command`에 **전체 `DriveCommand`**를 20ms마다 보냅니다. 제동 명령만 단독 송신하는 예제가 아닙니다.

| 명령 필드 | 발행 값 |
| --- | --- |
| `header.stamp` | 발행 시점의 ROS 시각 |
| `session_id` | 시작할 때 확인한 드라이버 세션 |
| `gear` | 중립(N), `2` |
| `speed_mps` | `0.0` m/s |
| `steering_angle_rad` | 시작 시 관측한 실제 조향각을 계속 유지, rad |
| `brake_percent` | 사용자가 지정한 비율, % |

드라이버는 이 명령을 `0x502` 조향·`0x503` 제동·`0x504` 기어/속도로 송신합니다. 예제는 조향을 중앙으로 바꾸지 않습니다. 시작 조향각이 유한하지 않거나 절댓값이 `0.3 rad`를 넘으면 실행을 거절합니다. 이 값은 예제의 보수적인 입력 범위이며 드라이버의 `max_steering_angle_rad` 설정을 바꾸지 않습니다. 드라이버를 더 좁은 범위로 설정했다면 드라이버의 거절 사유를 출력하고 중단합니다.

시작 전에 최대 5초 동안 다음 조건을 기다립니다.

- `hardware_drive`, CAN 송신 활성, `control_requested=true`, `drive_ready=true`, 다른 활성 구동 명령 없음.
- 301·303·304·10A 피드백 유효, A 선택, 차단 고장/범퍼/비상정지/병렬주행 없음.
- 중립(N), 실제 속도 절댓값 `0.05 m/s` 이하, 모드 AD(1) 또는 semi-autonomous(3).
- ROS 상태 토픽이 0.25초 이내에 수신되고 구독자가 연결됨.

실행 중 세션 변경·상태 수신 중단·기어/속도/A/고장 조건 상실을 확인하면 발행을 끝냅니다. 새 세션으로 자동 재시작하지 않습니다. 드라이버가 적용하는 정지 유지 시간·AD 모드 확인·명령 유효기간 등의 조건도 그대로 적용됩니다. 모드 3에서 시작했더라도 드라이버의 제한 시간 안에 명령 송신 이후의 새 AD 모드 피드백이 확인되지 않으면 드라이버가 명령을 무효화합니다.

종료 후 1.5초 동안 압력(Pa)과 `drive_active=false` 여부를 관측합니다. `drive_active`는 드라이버의 송신 의도이며 물리적 제동 해제를 뜻하지 않습니다. 정상 완료나 Ctrl+C로 client의 발행을 끝내면 드라이버는 명령 만료 후 제한된 시간 동안 `0x502/503/504 valid=0`을 보내고 구동 송신을 중단합니다. `0x501` 연속 송신은 유지됩니다. 이 동작은 자동 제동이나 차량 정지를 보장하지 않으며, 예제는 정지 성능을 판정하거나 정지 제동량을 산출하지 않습니다.

전체 명령 처리와 파라미터는 [드라이버 매뉴얼](../README.md)을 참고하세요.

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

## 상태 모니터 시작 파라미터

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

# Withus Python 예제

`withus_examples`는 드라이버의 공개 ROS 인터페이스를 사용하는 client 패키지입니다. 드라이버와 client를 각각 다른 터미널에서 실행합니다. 각 예제 launch는 해당 client 하나만 시작하며, 드라이버 실행·CAN 설정·제어 서비스 호출을 수행하지 않습니다.

| 예제 launch | 기능 |
| --- | --- |
| `status_monitor.launch.py` | 상태 요약과 수신 중단 감시 |
| `keyboard_teleop.launch.py` | WASD 주행·정지, 속도 조절과 정지 상태 기어 변경 |

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
cd ~/ros2_workspace/withus_ws
source install/setup.bash
ros2 launch withus_bringup withus_bringup.launch.py can_interface:=can0
```

드라이버는 `0x501 request=1`을 계속 보내며 조종기 A로 모드를 선택합니다. client의 유효한 `DriveCommand`가 없으면 구동 명령 `0x502/503/504`를 시작하지 않습니다.

## 상태 모니터: 터미널 2

```bash
cd ~/ros2_workspace/withus_ws
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

## 키보드 제어: 터미널 3

상태 모니터를 켜 둔 상태에서 별도 터미널에 실행합니다. 드라이버는 터미널 1의 한 개만 사용합니다.

```bash
cd ~/ros2_workspace/withus_ws
source install/setup.bash
ros2 launch withus_examples keyboard_teleop.launch.py
```

현재 터미널이 키보드 입력 창입니다. Linux의 제어 터미널(`/dev/tty`)을 사용하므로 일반 터미널이나 TTY를 할당한 SSH 세션에서 실행합니다. 입력 터미널이 없는 백그라운드 실행은 사용할 수 없습니다. 직접 실행도 가능합니다.

```bash
ros2 run withus_examples keyboard_teleop
```

직접 실행의 입력 timeout 기본값은 1초입니다. launch와 같은 10초를 쓰려면 `--key-timeout-sec 10.0`을 추가합니다.

### 고정 터미널 화면

키보드 예제는 실행한 터미널의 같은 위치를 초당 최대 10회 갱신합니다. 조작할 때마다 로그가 아래로 쌓이지 않으며 별도 창이나 GUI 패키지는 필요하지 않습니다.

- 조작 안내: `w/a/s/d`, `q/z`, `1/2/3/4`, Ctrl+C
- `Selected speed`: q/z로 지정한 설정 속도
- `Remembered gear`: 마지막으로 전환이 확인된 기어 선택값
- `Actual gear`: 차량 피드백으로 확인한 실제 기어
- 조종기 A 상태, 현재 명령의 속도·조향·제동, 준비·기어 대기·입력 만료 상태

설정 속도는 실제 주행 속도와 구분하고, 기억한 기어도 실제 기어와 별도로 표시합니다. 상태 수신이 만료된 실제 값은 `STALE` 또는 `--`로 표시합니다. 터미널 창 크기를 바꾸면 화면 폭과 높이에 맞춰 표시하며, 화면이 작아 잘린 항목은 창을 넓히면 다시 보입니다. ANSI 화면 제어를 지원하는 일반 Linux 터미널을 사용합니다.

현재 안내·거부 사유는 화면의 `Notice`에서 확인합니다. ROS 콘솔 로그는 숨기고 기존 ROS 로그 파일과 `/rosout`에는 계속 기록합니다. Ctrl+C로 종료하면 원래 터미널 화면과 입력 설정으로 복구합니다. 화면 갱신 주기는 명령 발행 주기와 별개이며 키 입력·기어 기억·명령 만료 동작을 바꾸지 않습니다.

### 키와 목표값

| 키 | 요청 |
| --- | --- |
| `w` | 선택 속도 적용, 현재 조향·기어 유지. 양의 속도에서는 제동 0% |
| `a` | 조향만 좌측 최대 설정값으로 변경. 현재 속도·제동·기어 유지 |
| `d` | 조향만 우측 최대 설정값으로 변경. 현재 속도·제동·기어 유지 |
| `s` | 속도 0, **설정 제동량(기본 50%), 조향 0**, 현재 명령 기어 유지 |
| `q` | 선택 속도 +0.1 m/s |
| `z` | 선택 속도 −0.1 m/s, 최솟값 0 |
| `1` / `2` / `3` / `4` | P / D / N / R 요청 |
| `Ctrl+C` | 명령 발행을 즉시 중단하고 종료 |

초기 선택 속도는 **0.1 m/s**입니다. `a/d`는 선택 속도와 관계없이 **조향만** 바꾸고 현재 명령의 속도·제동·기어를 유지합니다. `w`를 눌러야 선택 속도를 적용하며, 이때 조향을 중앙으로 바꾸지 않습니다. 따라서 `s` → `a`는 속도 0과 설정 제동량을 유지한 좌측 조향 요청이고, 이어서 `w`를 누르면 그 조향을 유지하며 주행합니다.

`q/z`는 정지 상태에서 속도 설정만 바꾸며 주행을 시작하지 않습니다. 이미 양의 속도를 요청 중이면 현재 속도 목표도 조절합니다. `z`로 0까지 줄여도 설정한 제동이나 조향 중립을 요청하지 않으며, 제동은 `s`로 요청합니다. 속도 0 이후 `q`로 선택 속도를 올려도 새 `w` 입력 전에는 출발하지 않습니다. R 기어도 음수 속도를 넣지 않으며 선택한 속력으로 후진합니다. P/N에서도 정지 조향은 가능하지만 양의 속도 `w` 요청은 거부합니다. 방향키는 지원하지 않습니다.

속도·조향 상한은 client 시작 시 같은 namespace의 `withus_driver` 파라미터를 한 번 읽어 적용합니다. 기본 상한은 D에서 1.0 m/s, R에서 0.5 m/s, 조향 절댓값 0.3 rad입니다. `q`를 계속 눌러도 해당 상한을 넘지 않으며 client가 드라이버 제한을 높이지 않습니다. 기어를 바꾸면 해당 방향의 상한을 적용합니다.

`a/d`는 조향각을 조금씩 누적하지 않고 최대 설정값으로 요청합니다. 기본 부호는 `a=+max_steering_angle_rad`, `d=-max_steering_angle_rad`입니다. 원본 매뉴얼에 실제 좌우 부호가 명시되어 있지 않으므로 정지 상태에서 방향을 확인합니다. 반대인 차량에서는 다음처럼 부호를 바꿉니다.

```bash
ros2 launch withus_examples keyboard_teleop.launch.py left_steering_sign:=-1
```

정지 상태의 방향 확인은 아래 준비 절차를 마친 뒤 `s` → `a/d` 순서로 조작합니다. 선택 속도를 0으로 바꿀 필요 없이 속도 0과 기존 설정 제동량을 유지하며 조향만 요청합니다. 확인 뒤 `s`로 중앙을 요청합니다. 입력 timeout은 조향을 중앙으로 바꾸지 않고 발행을 중단합니다.

### 조작 순서

1. 상태 모니터에서 최신 피드백과 실제 정지를 확인하고 조종기 A를 선택합니다.
2. 키보드 창에서 `s`를 눌러 준비합니다. 현재 기어, 속도 0, 설정 제동량(기본 50%), 조향 0의 첫 명령을 보내고 모드 확인을 기다립니다. 실행 직후 바로 `w/a/d`를 눌러도 명령을 저장해 두었다가 출발하지 않습니다.
3. 새 피드백으로 AD 모드가 확인되면 원하는 기어 키를 **한 번** 누릅니다. 예를 들어 `2`는 D입니다. 정지 상태에서 기어 키로 시작하면 현재 기어로 준비하고 AD 확인과 정지 유지가 끝난 뒤 요청 기어를 적용합니다.
4. 기어 확인을 마친 뒤 **새 `w` 입력**으로 주행하고 `a/d`로 조향합니다. `a/d`만으로는 정지 상태에서 출발하지 않습니다. 상태 모니터의 실제 기어·속도·조향을 함께 확인합니다.
5. `s`로 정지를 요청합니다. 다른 기어로 변경하려면 실제 속도가 드라이버의 정지 기준 이내로 유지된 뒤 기어 키를 다시 누릅니다. 주행 중의 기어 변경 입력은 예약하지 않고 거부합니다.

기어 응답을 기다리는 동안에는 속도 0과 설정 제동량을 유지합니다. 최초 기어 요청은 AD 확인이나 정지 유지 대기 중에 지우지 않으므로 전환을 시작하려고 같은 숫자를 다시 누를 필요는 없습니다. 입력 timeout은 대기 중에도 적용하며, 같은 기어 키를 다시 누르면 입력 유효시간만 갱신하고 드라이버의 기어 응답 제한 시간을 늘리지는 않습니다.

AD 확인·정지 유지 중으로 아직 새 기어 요청을 보내기 전이라면 `s`로 예정된 기어 변경을 취소하고 현재 기어에서 정지를 요청합니다. 기억한 기어로 재준비하는 중에도 `s`를 다시 누르면 예정된 복원을 취소하고 현재 기어를 새 선택으로 기억합니다. 이미 새 기어 요청을 보낸 뒤에는 `s`를 눌러도 그 요청 기어를 다른 값으로 바꾸지 않습니다. 새로운 기어 피드백과 `drive_gear_pending=false`를 확인한 다음 새 `w` 입력으로 주행합니다. 활성 제어 중 `w/a/d/q/z/s`는 확인한 기어를 유지합니다. 정상 입력 timeout 뒤에는 그 기어만 기억하며, 아래 순서로 새 준비 입력에서 다시 적용합니다. 명령을 발행하지 않는 동안 실제 차량의 기어를 유지시키는 기능은 아닙니다.

시작 시에는 `hardware_drive`, 송신 활성, 최신 `VehicleStatus`·`DriverStatus`, 유효한 10A/301/303/304, A 선택, 차단 고장 없음과 정지 유지 조건을 확인합니다. 모드 확인은 `drive_mode_confirmed=true`뿐 아니라 실제 모드 1과 AD 신호 수신 상태를 함께 사용합니다. `drive_active=true`만으로 출발 가능한 상태라고 판단하지 않습니다.

### 입력 중단과 종료

마지막 유효한 조작 키 이후 `key_timeout_sec`(launch 기본 **10.0초**, 직접 실행 기본 **1.0초**)가 지나면 **모든 명령 발행을 중단하고 세션·이전 목표·미확인 기어 요청을 지웁니다.** 정상 입력 timeout에서는 마지막으로 새 피드백을 확인한 기어만 client 메모리에 남깁니다. `s`와 달리 자동 제동이나 조향 중립 명령을 만들지 않습니다. 이 timeout은 주행뿐 아니라 준비·기어 대기·정지 명령에도 적용됩니다. 계속 제어하려면 키를 누르고 있거나 반복 입력하여 갱신합니다. 키보드 반복 입력 시작 간격보다 짧게 timeout을 설정하면 키를 누르고 있어도 발행이 중단될 수 있습니다.

키 입력 감시와 드라이버의 명령 timeout은 별개입니다. 입력이 유효한 동안 client는 새 timestamp의 `DriveCommand`를 20ms마다 발행합니다. 발행이 끊기면 드라이버는 마지막 명령 수락 후 기본 200ms에 목표와 세션을 폐기하고, 기본 1초 동안 `0x502/503/504 valid=0`을 보낸 뒤 구동 송신을 멈춥니다. **발행 중단과 valid=0은 실제 제동·정지 완료를 뜻하지 않습니다.** 일반 정지는 `s`로 명시적으로 요청하고 실제 응답을 확인하세요. 설정 제동량은 이 예제의 요청값이며 실제 정지 성능의 보증값은 아닙니다.

정상 입력 timeout 뒤에는 드라이버가 다시 준비된 상태에서 새 `s` 또는 기어 키로 시작합니다.

1. 새 `s`는 현재 관측 기어, 속도 0, 설정 제동량(기본 50%), 조향 0으로 새 세션을 준비합니다.
2. 새 AD 모드와 정지 유지가 확인되면 기억한 마지막 확인 기어가 현재 기어와 다를 때 다시 요청합니다. 예를 들어 D를 사용하다 발행 중단 후 실제 기어가 N으로 보고되면, 새 `s`에서 N으로 준비한 다음 D를 다시 요청합니다. 숫자 `2`를 반복할 필요는 없습니다.
3. 새 기어 피드백을 확인한 뒤 **새 `w` 입력**으로 주행합니다. 이전 속도·조향 목표는 복원하지 않습니다. 새 숫자 키로 준비를 시작하면 기억한 기어 대신 새 선택을 사용합니다.

이 기억은 정상 입력 timeout에만 적용합니다. A 해제·상태 수신 중단·고장 등의 조건 상실이나 예상하지 못한 세션 변경에서는 기어 기억도 지우며, 조건 회복만으로 이전 목표를 발행하지 않습니다. 기어를 기억하는 동안에도 client 발행은 중단되어 있습니다. Ctrl+C는 추가 제동 명령 없이 즉시 발행을 중단하고 종료하며, client를 다시 실행하면 이전 기어 기억은 남지 않습니다.

client 종료는 드라이버를 종료하거나 `set_control(false)`를 호출하지 않습니다. 터미널 1의 드라이버는 `0x501` 요청을 계속 송신합니다. 하나의 차량에는 키보드 등 주행 명령을 보내는 client를 하나만 실행합니다. 상태 모니터는 동시에 실행할 수 있습니다.

### 키보드 launch 인자

| 인자 | 기본값 | 의미 |
| --- | --- | --- |
| `namespace` | `withus` | 드라이버 노드와 상태·명령 토픽 namespace |
| `key_timeout_sec` | `10.0` | 마지막 조작 입력 이후 발행 중단까지의 시간, 양수 초. 직접 실행의 기본값은 `1.0` |
| `initial_speed` | `0.1` | 시작 시 선택 속도, m/s |
| `speed_step` | `0.1` | q/z 한 번당 속도 증감량, m/s |
| `stop_brake_percent` | `50.0` | s·최초 준비·재준비·기어 전환의 제동 비율, 유한한 0~100% |
| `left_steering_sign` | `1` | a의 조향 부호. `1` 또는 `-1` |

```bash
ros2 launch withus_examples keyboard_teleop.launch.py \
  key_timeout_sec:=10.0 initial_speed:=0.1 speed_step:=0.1 \
  stop_brake_percent:=50.0
```

`initial_speed`는 유한한 0 이상 값, `speed_step`과 `key_timeout_sec`는 유한한 양수여야 합니다. `stop_brake_percent`는 유한한 0~100 값이며, `0`은 제동을 요청하지 않는 값입니다. 소수도 입력할 수 있지만 CAN 제동 필드는 1% 단위이므로 드라이버 인코더에서 가장 가까운 정수 비율로 반올림합니다. 값을 실행 중에 바꾸려면 client를 새 인자로 다시 실행합니다.

직접 실행에서는 같은 값을 CLI 인자로 지정합니다. 제동 기본값은 두 실행 방식 모두 50%이며 입력 timeout만 launch 10초·직접 실행 1초로 다릅니다.

```bash
ros2 run withus_examples keyboard_teleop \
  --stop-brake-percent 50.0 --key-timeout-sec 10.0
```

이 제동 설정은 `s`, 최초 준비, idle 후 재준비와 기어 전환에 동일하게 적용합니다. `a/d`는 현재 요청 제동량을 유지하고 양의 속도 `w`는 제동 0%를 사용합니다. 입력 timeout과 Ctrl+C는 설정값과 관계없이 추가 제동 없이 발행을 중단합니다. 키보드 client 인자이며 드라이버 YAML의 가상 차량용 `stop_brake_percent`와 별개입니다.

예제는 드라이버의 속도·조향 제한과 정지 판정 파라미터를 읽어 사용합니다. 드라이버 제한 변경은 [기본 YAML](../withus_bringup/config/withus_driver.yaml)에서 설정한 뒤 드라이버와 키보드 client를 모두 다시 실행해 새 제한값을 읽습니다. 전체 명령 처리와 메시지 의미는 [드라이버 매뉴얼](../README.md)을 참고하세요.

### 피드백 거부로 제어가 중단될 때

`status_304_rejected_frames`가 늘면서 `status_304_valid=false`가 되면 원시 CAN과 제동 설정을 확인합니다. 현재 디코더는 8MPa를 넘는 제동압이 담긴 `0x304`를 거부하므로 ROS 압력 값만으로 초과 여부를 확인하지 못할 수 있습니다. 키보드의 `stop_brake_percent`를 필요한 값으로 설정하고, 정상 피드백·준비 상태가 회복된 뒤 새 `s` 또는 기어 키로 다시 준비합니다. 자세한 동작은 [304 수신 거부와 제어 중단](../README.md#304-수신-거부와-제어-중단-확인)을 참고하세요.

## 상태 모니터 표시 내용

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

## 상태 모니터 코드 구성

- `on_status()`: 최신 메시지와 로컬 수신 시각만 저장합니다. 구독 QoS는 Reliable / Volatile / KeepLast(1)입니다.
- `on_timer()`: ROS 메시지 중단을 판단하고, 연결된 동안 상태 요약을 출력합니다. ROS 시뮬레이션 시간이 멈춰도 감시 타이머는 동작합니다.
- `describe_group()`: ID별 received/valid를 확인한 뒤 상태값을 표시합니다.
- `main()`: 단일 스레드 executor로 콜백을 실행하고 종료 시 노드를 정리합니다.

콜백에서 매 프레임 로그를 출력하지 않고 최신 메시지만 저장하므로 수신 빈도와 출력 빈도를 분리합니다. Python 예제에 CAN 디코딩이나 드라이버 내부 라이브러리 의존성은 없습니다.

# Withus ROS 2 Driver

Withus 플랫폼의 CAN 데이터를 수신하는 C++17 기반 ROS 2 드라이버입니다.
현재는 Linux SocketCAN으로 프레임을 수신하고, `0x304` 프레임의 차량 속도를 해석해 ROS 노드 로그로 출력합니다.

**작성 기준: 2026-09-21 작업 파일.** 이 README는 현재 구현과 실행 방법을 설명합니다.
차량 상태 토픽 발행, CAN 송신, 주행 제어, 고객용 예제 패키지는 아직 구현되지 않았습니다.

## 패키지 구성

| 패키지 | 현재 구성 |
| --- | --- |
| [withus_driver](withus_driver/) | SocketCAN 통신 라이브러리, 속도 디코더, 단일 프레임 수신 확인 프로그램, 수신 스레드를 사용하는 ROS 노드 |
| [withus_bringup](withus_bringup/) | 드라이버 실행용 Python launch 파일, CAN 설정 스크립트, 설치 규칙과 실행 의존성 |
| [withus_msgs](withus_msgs/) | `ament_cmake` 패키지 골격. 메시지·서비스 정의 없음 |

```text
withus_driver_ros2/
├── README.md
├── docs/withus_CAN_datasheet.pdf
├── withus_driver/
│   ├── include/withus_driver/
│   │   ├── can_frame.hpp
│   │   ├── socket_can_transport.hpp
│   │   ├── withus_protocol.hpp
│   │   └── withus_driver_node.hpp
│   ├── src/
│   │   ├── socket_can_transport.cpp
│   │   ├── can_connection_check.cpp
│   │   ├── withus_protocol.cpp
│   │   ├── withus_driver_node.cpp
│   │   └── main.cpp
│   ├── CMakeLists.txt
│   └── package.xml
├── withus_bringup/
│   ├── launch/withus_bringup.launch.py
│   ├── scripts/setup_can.sh
│   ├── CMakeLists.txt
│   └── package.xml
└── withus_msgs/
    ├── CMakeLists.txt
    └── package.xml
```

## 준비 및 빌드

Linux SocketCAN, ROS 2 Humble, C++17 컴파일러를 사용합니다.
아래 명령은 ROS 2, `colcon`, `rosdep` 설치 및 초기 설정이 완료된 환경을 기준으로 합니다.
워크스페이스 경로는 사용하는 환경에 맞게 변경하세요.

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_workspace/withus_ws

rosdep install --from-paths src --ignore-src --rosdistro humble -y
colcon build --packages-up-to withus_bringup
source install/setup.bash
```

`withus_bringup/package.xml`에는 `withus_driver`, `launch`, `launch_ros`, `ros2launch`, `ros2run`, `iproute2`, `can-utils` 실행 의존성이 선언돼 있습니다.
`rosdep install`이 필요한 시스템 패키지를 설치하고, `colcon build --packages-up-to withus_bringup`이 드라이버와 bringup을 빌드합니다.
`iproute2`는 CAN 설정용 `ip` 명령, `can-utils`는 프레임 확인용 `candump` 등의 도구를 제공합니다.

새 터미널에서는 워크스페이스 루트에서 `source install/setup.bash`를 실행한 뒤 아래 명령을 사용하세요.

## CAN 인터페이스 설정

현재 확인한 하드웨어 환경은 GS_USB CAN 어댑터의 `can0`, 통신 속도 `500000 bit/s`입니다.
연결된 인터페이스를 먼저 확인합니다.

```bash
ip -details link show type can
```

설정이 필요하면 드라이버 실행 전에 [setup_can.sh](withus_bringup/scripts/setup_can.sh)를 실행합니다.
기본값은 `can0`, `500000 bit/s`이며, 설정 명령에서 필요한 `sudo` 비밀번호를 입력합니다.

```bash
ros2 run withus_bringup setup_can.sh
```

인터페이스와 통신 속도를 명시하려면 다음처럼 사용합니다.

```bash
ros2 run withus_bringup setup_can.sh can0 500000
```

| 인자 | 기본값 | 의미 |
| --- | --- | --- |
| 첫 번째 | `can0` | 설정할 CAN 인터페이스 |
| 두 번째 | `500000` | 통신 속도, bit/s 단위 |

스크립트는 인자와 장치를 확인한 뒤 다음 순서로 실행합니다.

```text
CAN 인터페이스 확인 → down → bitrate 설정 → up → 상세 상태와 통계 출력
```

인터페이스를 내리는 동안 CAN 통신이 중단됩니다. 이미 원하는 속도로 활성화돼 있으면 재설정할 필요가 없습니다.
중간 명령이 실패하면 스크립트는 중단하며, 이전 설정으로 자동 복구하지 않습니다.

프레임을 직접 확인할 때는 다음 명령을 사용하고, `Ctrl+C`로 종료합니다.

```bash
candump can0
```

노드와 launch 파일은 준비된 인터페이스를 사용하며, bitrate 설정이나 인터페이스 활성화를 수행하지 않습니다.

## 실행 방법

### 단일 프레임 수신 확인

```bash
ros2 run withus_driver can_connection_check can0
```

소켓을 열고 최대 1초 동안 프레임 하나를 기다립니다.
수신하면 ID, 길이, 데이터를 출력한 뒤 소켓을 닫고 종료합니다.
이 프로그램은 ROS 노드를 생성하지 않는 일반 C++ 실행 파일입니다.

실제 CAN 환경에서 확인한 출력 예시:

```text
Interface: can0
CAN socket opened on: can0
Received CAN frame
ID: 0x142
Length: 8
Data:  00 00 01 06 00 28 04 2b
CAN socket closed.
```

종료 코드는 성공 시 `0`, 연결·수신 실패나 타임아웃 시 `1`, 인자 개수 오류 시 `2`입니다.

### 드라이버 노드 실행

```bash
ros2 run withus_driver withus_driver_node --ros-args -p can_interface:=can0
```

노드 이름은 `/withus/withus_driver`입니다.
별도 스레드에서 CAN 프레임을 계속 수신하며, 유효한 `0x304` 프레임의 속도를 m/s 단위로 출력합니다.
속도 로그는 최대 1초에 한 번 출력하도록 제한돼 있습니다.

정지 속도 값이 수신됐을 때의 예상 로그 본문:

```text
CAN socket opened on : can0
Speed: 0.00 m/s
```

`Speed` 로그는 속도 디코딩이 성공했을 때만 출력됩니다.
연결 로그만 출력되는 경우에는 `0x304` 프레임의 수신 여부와 형식도 확인해야 합니다.
`Ctrl+C`로 종료하면 수신 스레드 종료를 기다린 뒤 CAN 소켓을 닫습니다.

| 파라미터 | 타입 | 기본값 | 동작 |
| --- | --- | --- | --- |
| `can_interface` | string | `can0` | 시작 시 사용할 CAN 인터페이스. 실행 중 변경 불가 |

### Launch 파일로 실행

[withus_bringup.launch.py](withus_bringup/launch/withus_bringup.launch.py)는 같은 드라이버 노드에 `can_interface`를 전달합니다.
직접 노드 실행과 launch 실행 중 한 방법을 선택하세요.

빌드와 환경 설정을 마친 터미널에서 실행합니다.

```bash
ros2 launch withus_bringup withus_bringup.launch.py can_interface:=can0
```

`can_interface` 인자를 생략하면 `can0`를 사용합니다.
현재 파일 이름은 `withus_bringup.launch.py`입니다.

CAN 설정이 성공한 뒤 노드를 실행하려면 두 명령을 연결할 수 있습니다.

```bash
ros2 run withus_bringup setup_can.sh can0 500000 && \
ros2 launch withus_bringup withus_bringup.launch.py can_interface:=can0
```

사용하는 인터페이스를 바꿀 때는 스크립트 인자와 launch 인자에 같은 이름을 지정하세요.

## 구현 동작

- **통신:** `SocketCanTransport`가 소켓 열기·닫기와 Classical CAN 프레임 수신을 담당합니다. `poll()`로 기다린 뒤 `recv()`로 프레임 하나를 읽습니다.
- **프레임:** ID, 길이, 최대 8바이트 payload, 확장·Remote·Error 플래그를 `CanFrame`에 저장합니다.
- **프로토콜:** ROS에 의존하지 않는 `decode_vehicle_speed()` 함수로 속도를 해석합니다.
- **노드:** ROS 콜백은 `rclcpp::spin()`으로 처리하고, CAN 수신은 별도 `std::thread`에서 수행합니다.
- **종료:** 종료 플래그 설정 → 수신 스레드 `join()` → 소켓 닫기 순서로 정리합니다.

속도 디코더는 표준 데이터 프레임 `0x304`, 길이 8바이트만 처리합니다.
`data[0]`, `data[1]`을 little-endian unsigned 16비트 값으로 결합합니다.

```text
raw_speed = data[0] | (data[1] << 8)
speed_kmh = raw_speed × 0.1 - 80.0
speed_mps = speed_kmh / 3.6
```

예를 들어 첫 두 바이트가 `20 03`이면 raw 값은 800이고 결과는 0 m/s입니다.
계산한 속도가 -80~80 km/h 범위를 벗어나면 디코딩에 실패하며, 출력 인자를 갱신하지 않습니다.
신호 정의는 [CAN 데이터시트](docs/withus_CAN_datasheet.pdf) 11페이지를 참고하세요.

수신 루프의 `receive(frame, 100)`에서 100ms는 데이터가 없을 때의 대기 시간입니다.
타임아웃이면 다시 수신을 시도합니다. `ReceiveResult::Error`이면 오류 로그를 남기고 **수신 루프만 종료**하며, ROS 노드는 계속 실행됩니다.
현재 자동 재연결이나 수신 재시작 처리는 없습니다.

## 확인한 범위

| 항목 | 확인 내용 |
| --- | --- |
| CAN 연결과 단일 프레임 수신 | 사용자 실차 환경에서 `can0`, 500 kbps로 ID·길이·payload 출력 확인 |
| C++ 드라이버 빌드 | `70ce3b6` 기준 ROS 2 Humble에서 `-Werror`를 추가한 빌드·설치 통과 |
| Bringup 빌드와 설치 | 2026-09-21 별도 임시 경로에서 빌드·설치 후 `ros2 pkg executables withus_bringup`으로 `setup_can.sh` 등록 확인 |
| CAN 설정 스크립트 | `bash -n`으로 문법 확인. 실제 인터페이스 설정 실행은 아직 검증하지 않음 |
| Launch 파일 로드 | 설치한 패키지의 `ros2 launch withus_bringup withus_bringup.launch.py --show-args`로 파일 로드와 `can_interface=can0` 기본값 확인 |
| ROS 노드 속도 로그와 launch를 통한 실제 CAN 수신 | 아직 실행 결과를 확인한 기록 없음 |

Launch 파일 로드 확인은 CAN 노드를 실행하지 않고 수행했습니다.

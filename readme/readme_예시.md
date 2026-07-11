# 바이오뱅크 검체 이송 관리 자동화 시스템 (bio_transport)

바이오뱅크 워크셀에서 **RACK 기반 이송**과 **튜브(절대좌표) 이송**을 ROS 2 Action 기반으로 통합 제어하는 프로젝트입니다.  
UI(데스크탑 앱) → 메인 오케스트레이터 → 로봇 제어 서버(두산 로봇/시뮬레이터) 구조로 구성되어 있습니다.

---
## 1) 시스템 설계 / 통신 구조

### 노드 구성(런치 기준)

- **bio_ui** (`bio_transport.ui_integrated`)
  - PySide6 기반 UI
  - 액션 전송:
    - `/bio_main_control` (BioCommand) : RACK 명령
    - `/tube_main_control` (BioCommand) : TUBE/EMERGENCY/HOME 명령
- **bio_main** (`bio_transport.main_integrated`)
  - UI 명령 파싱/검증, 동작 순서 제어(락 기반)
  - 하위 서버 호출:
    - `/robot_action` (RobotMove)
    - `/tube_transport` (TubeTransport)
  - 브로드캐스트:
    - `/bio_emergency` (std_msgs/Bool)
- **bio_sub** (`bio_transport.rack_transport_action`)
  - 두산 로봇 제어(DSR_ROBOT2) + SAFE 상태 감지 및 HOME 복구 로직
  - 제공 액션:
    - `/robot_action` (RobotMove) : RACK/HOME 동작
    - `/tube_transport` (TubeTransport) : 튜브 절대좌표 이송

### 전체 플로우(요약)
![alt text](Flow_chart.png)


### 명령 포맷(대표)

- **RACK**
  - `RACK,<CMD>,<SRC>,<DST>`
  - 예: `RACK,IN,NONE,A-2`
- **TUBE**
  - `TUBE,<MODE>,<SRC>,<DST>`
  - 예: `TUBE,IN,NONE,A-2-1`
- **긴급정지/홈 복귀(UI 버튼)**
  - `EMERGENCY,STOP,NONE,NONE`
  - `HOME,NONE,NONE,NONE`

> 좌표/스테이션 정의는 `bio_transport/rack_stations.py` 및 메인 오케스트레이터 내부 맵핑 테이블에서 관리합니다.

---

## 2) 운영체제 환경

권장/테스트 기준(예시):

- **OS**: Ubuntu 22.04 LTS
- **ROS 2**: Humble Hawksbill
- **Python**: 3.10.x (ROS 2 Humble 기본)
- **로봇 드라이버/패키지**
  - `dsr_bringup2` (런치에서 include)
  - `DSR_ROBOT2` (Python API, 두산 로봇 제어)

---

## 3) 사용한 장비 목록

- **로봇**: Doosan Robotics **M0609** (코드 기본값: `ROBOT_MODEL="m0609"`)
- **컨트롤/네트워크**
  - 로봇 컨트롤러(이더넷 연결)
  - ROS 2 실행 PC(워크스테이션)
- **엔드이펙터**
  - DO 기반 그리퍼 (기본 DO 채널: 1/2)  
    - 구현: `bio_transport/gripper_io.py`
- **워크셀 구성 요소(프로젝트 범위)**
  - 랙(A-1~B-3 등), 워크벤치/튜브 포인트

---

## 4) 의존성 (requirements.txt)

이 프로젝트는 ROS 2 패키지(apt) + Python 패키지(pip)를 함께 사용합니다.

### ROS 2 (apt) 예시
- `rclpy`
- `std_msgs`
- `launch`, `launch_ros`
- `ament_index_python`
- (두산 드라이버) `dsr_bringup2`, `DSR_ROBOT2` 제공 패키지

### Python (pip) 예시: `requirements.txt`
```txt
PySide6>=6.5
```

> UI는 PySide6 기반입니다. (리눅스에서 패키지 설치 방식은 환경에 맞게 선택)

---

## 5) 간단 사용 설명 (launch 순서 및 스크립트)

### (A) 워크스페이스 빌드

```bash
# ROS 2 환경
source /opt/ros/humble/setup.bash

# 워크스페이스 예시
mkdir -p ~/bio_transport_ws/src
cd ~/bio_transport_ws/src

# 이 저장소의 src/ 내용을 복사/클론했다고 가정
cd ~/bio_transport_ws
colcon build --symlink-install
source install/setup.bash
```

### (B) 통합 런치 실행 (권장)

```bash
ros2 launch bio_transport bio_integrated.launch.py   mode:=real host:=192.168.1.100   dry_run:=false skip_probe:=false
```

- `mode`: `real` / `virtual` (환경에 맞게)
- `host`: 로봇 컨트롤러 IP(또는 시뮬레이터 호스트)
- `dry_run`: 로봇 동작을 실제로 수행하지 않고 흐름만 테스트
- `skip_probe`: (현재 sub 노드에서 미사용일 수 있음)

### (C) 개별 실행(디버깅용)

> 실행 순서: (두산 bringup) → **bio_sub** → **bio_main** → **bio_ui**

```bash
# 로봇 제어 서버
ros2 run bio_transport bio_sub --ros-args -p dry_run:=true

# 메인 오케스트레이터
ros2 run bio_transport bio_main

# UI
ros2 run bio_transport bio_ui
```

---

## 참고: 인터페이스(Action) 패키지

- 액션 타입: `BioCommand`, `RobotMove`, `TubeTransport`
- 본 저장소에는 액션 정의 폴더가 포함되어 있으며, Python 노드에서는 `biobank_interfaces.action` 을 import하도록 작성되어 있습니다.  
  환경에 따라 **인터페이스 패키지명(폴더/패키지.xml/CMake 프로젝트명)을 하나로 통일**해서 사용하는 것을 권장합니다.

---

## 라이선스

- 패키지 라이선스: `bio_transport/LICENSE` 참고

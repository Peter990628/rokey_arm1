
# 💊 약국 업무 보조 시스템
> **조 이름:** [C-2 - ROKEY]
> **팀원:** [김혜승_박현정_서영채_조해벽]

## 1. 🎨 시스템 설계 및 플로우 차트
프로젝트의 전체적인 구조와 소프트웨어 흐름도입니다.

### 1-1. 시스템 설계도 (System Architecture)
<p align="center">
  <img src="./images/system_design.png" alt="시스템 설계도 이미지" width="400">
</p>
* *설명: PC와 매니퓰레이터 간의 통신 구조를 나타냅니다.*

### 1-2. 플로우 차트 (Flow Chart)
<p align="center">
 <img width="1531" height="2031" alt="협동1_플로우차트" src="https://github.com/user-attachments/assets/9dffe982-0576-4049-88eb-5fdefcf9f6e8" />
</p>
* *설명: [UI부터 전체 프로세스 진행도를 나타냅니다.]*

---

## 2. 🖥️ 운영체제 환경 (OS Environment)
이 프로젝트는 다음 환경에서 개발하였습니다.

* **OS:** Ubuntu 22.04 LTS
* **ROS Version:** ROS2 Humble
* **Language:** Python 3.10.12
* **IDE:** VS Code

---

## 3. 🛠️ 사용 장비 목록 (Hardware List)
프로젝트에 사용된 주요 하드웨어 장비입니다.

| 장비명 (Model) | 수량 | 비고 |
|:---:|:---:|:---|
| 로봇 팔 | 1 | [Doosan Robotics M0609, DO 기반 그리퍼] |
| PC | 3 | [로봇 제어, UI, DB] |
| 제조기 서랍장 | 1 | [다이소 제품] |
| 툴 거치대 | 1 | [3D 프린팅] |
| 스크래퍼 | 1 | [3D 프린팅] |
| 약 병 오프너 | 1 | [3D 프린팅] |
| 약 적재소 | 1 | [선반] |
---

## 4. 📦 의존성 (Dependencies)
이 프로젝트는 ROS 2 패키지(apt) + Python 패키지(pip)를 함께 사용합니다.

### 🤖 ROS2 패키지 (rosdep/apt로 설치)
- `ros-humble-rclpy`
- `ros-humble-std-msgs`
- `ros-humble-action-msgs`

### 🦾 Doosan Robotics 패키지 
다음 패키지는 Doosan Robotics ROS 2 저장소를 워크스페이스에 설치하고
`colcon build`하여 사용합니다.

- `dsr_bringup2`
- `dsr_common2`
- `dsr_msgs2`

`dsr_common2`는 로봇 제어 코드에서 사용하는 다음 Python 모듈을 제공합니다.

- `DR_init`
- `DSR_ROBOT2`
- `DR_common2`

### 📁 Python 라이브러리 (pip로 설치)
- `requests`
- `Flask`
- `Flask-SocketIO`
- `Django`
- `djangorestframework`
- `pymodbus`
- `django-cors-headers`
- `psycopg2-binary`

> Python 3.10 이상 환경을 권장합니다.

---
## 5. 🏗️ 개별 실행 (디버깅 용)

### 5-1. ROS 2 워크스페이스 빌드

```bash
cd ~/peterws/ws_cobot_pjt/ws_dsr

source /opt/ros/humble/setup.bash

colcon build --symlink-install --packages-select rokey
source install/setup.bash
```
### 5-2. Django 백엔드 실행
```bash
cd ~/peterws/ws_cobot_pjt/ws_dsr/src/rokey/pharmacy_backend

python3 manage.py migrate
python3 manage.py runserver 0.0.0.0:8000
```
migrate는 데이터베이스 마이그레이션이 필요한 경우에 실행합니다.
백엔드 API 확인:
```bash
curl http://localhost:8000/api/events/
curl http://localhost:8000/api/medicine/
```
다른 PC에서 백엔드에 접근하는 경우 localhost 대신 백엔드 PC의 IP를 사용합니다.

### 5-3. 웹 UI 실행
웹 UI는 실행 전에 Django 백엔드가 동작하고 있어야 합니다.
```bash
cd ~/peterws/ws_cobot_pjt/ws_dsr/src/rokey

source /opt/ros/humble/setup.bash
source ~/peterws/ws_cobot_pjt/ws_dsr/install/setup.bash

python3 pjt_front_ui/app.py
```
웹 브라우저에서 다음 주소로 접속합니다.
```bash
http://localhost:5000
```
다른 PC에서 접속하는 경우:
```bash
http://<UI 실행 PC IP>:5000
```
pjt_front_ui/app.py의 DB_API_BASE_URL은 실제 Django 백엔드 주소와 같아야 합니다.
### 5-4. 두산 로봇 Bringup 실행
```bash
source /opt/ros/humble/setup.bash
source ~/peterws/ws_cobot_pjt/ws_dsr/install/setup.bash

ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py \
  mode:=real \
  host:=192.168.1.100 \
  port:=12345 \
  model:=m0609
```
host는 실제 로봇 컨트롤러 IP에 맞게 변경합니다.
### 5-5. 매니퓰레이터 작업_2 테스트
```bash
source /opt/ros/humble/setup.bash
source ~/peterws/ws_cobot_pjt/ws_dsr/install/setup.bash

ros2 run rokey manipulator_test_2
```
종이봉투 동작만 테스트:
```bash
ros2 run rokey manipulator_test_2 --paper-only
```
스크래퍼 동작만 테스트:
```bash
ros2 run rokey manipulator_test_2 --scraper-only
```
작업 완료 토픽 확인:
```bash
ros2 topic echo /dsr01/task_done
```
### 5-6. Task Manager Bridge 실행(선택)
Django 백엔드 정보를 ROS 2 토픽으로 확인하려면 다음 노드를 실행합니다.
```bash
ros2 run rokey task_manager_bridge --ros-args \
  -p backend_base_url:=http://<백엔드_PC_IP>:8000/api
  ```
발행 토픽 확인:
```bash
ros2 topic echo /dsr01/pharmacy/events
ros2 topic echo /dsr01/pharmacy/medicine
ros2 topic echo /dsr01/pharmacy/refill_required_medicine
```
추천 실행 순서는 다음과 같습니다.

```text
Django 백엔드
├── pjt_front_ui
└── task_manager_bridge

두산 Bringup
└── manipulator_test_2
```

---
## 6. ▶️ 실행 순서 (Usage Guide) (작성중)
프로젝트를 실행하기 위한 순서입니다. 터미널 명령어를 순서대로 입력해 주세요.
### Step 1. 로봇 초기화 - 로봇의 전원을 켜고 통신을 연결합니다.
```bash
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=real host:=192.168.1.100 port:=12345 model:=m0609
```
### Step 2. 메인 제어 노드 실행 - 기능 알고리즘을 시작합니다.
```bash
ros2 run rokey_arm1 main_control.py
```

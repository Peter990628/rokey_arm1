
# 💊 약국 업무 보조 시스템
> **조 이름:** [C-2 - ROKEY]
> **팀원:** [김혜승_박현정_서영채_조해벽]

## 1. 🎨 시스템 설계 및 플로우 차트
프로젝트의 전체적인 구조와 소프트웨어 흐름도입니다.

### 1-1. 시스템 설계도 (System Architecture)
<p align="center">
  <img width="4398" height="6322" alt="협동1_시스템아키텍처" src="https://github.com/user-attachments/assets/7557a425-36a0-494f-8d38-b805b70bbfb5" />
</p>
https://drive.google.com/file/d/1rQMJtL6ISLpyqz9C98pV-1yGLuqX5OQS/view?usp=sharing

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
| 약병 오프너 | 1 | [3D 프린팅] |
| 약병 고정틀 | 1 | [3D 프린팅] |
| 약 적재소 | 1 | [선반] |
<details>
  <summary>📸 사용 장비 실제 사진 보기 (클릭하여 펼치기)</summary>
  
  #### Doosan M0609
  <img width="2252" height="4000" alt="IMG_20260713_175347" src="https://github.com/user-attachments/assets/6c2c5e06-3799-4459-9d04-16c5c0604831" />

  #### 그리퍼 툴
  <img width="2252" height="4000" alt="IMG_20260713_175357" src="https://github.com/user-attachments/assets/f5e704d9-a003-4fa7-8b38-fc74972df3f4" />

  #### 제조기 서랍장
  <img width="2252" height="4000" alt="20260713_171253" src="https://github.com/user-attachments/assets/aadff99e-1969-49fa-a882-1434a668eb19" />

  #### 툴 거치대
  <img width="2252" height="4000" alt="20260713_171218" src="https://github.com/user-attachments/assets/8b37e0db-808e-4fbd-a95c-596e435f59b3" />
  
  #### 스크래퍼
  <img width="2252" height="4000" alt="IMG_20260713_175222" src="https://github.com/user-attachments/assets/daa9cc56-0900-4d0b-a873-d22895193a1c" />
  
  #### 약병 오프너
  <img width="2252" height="4000" alt="20260713_174941" src="https://github.com/user-attachments/assets/9b92996c-56ca-43a4-9715-9b10ef94d329" />

  #### 약병 고정틀
  <img width="4000" height="2252" alt="20260713_171234" src="https://github.com/user-attachments/assets/bc6dc640-6042-464e-9574-1cf5a15edeba" />

  
</details>

<p align="center">
<img width="894" height="665" alt="스크린샷 2026-07-13 17-17-45" src="https://github.com/user-attachments/assets/9b5aa929-602f-4e30-9a34-23a4cf961427" />
</p>
* *설명: [지도를 나타냅니다..]*

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
## 6. ▶️ 통합 실행 순서 (Usage Guide)

아래는 **Backend PC, UI PC, Robot Control PC**를 분리해 실행하는 기준입니다. 한 PC에서 모두 실행하는 경우 `<BACKEND_PC_IP>`는 `127.0.0.1`로 사용할 수 있습니다. 명령어의 `<BACKEND_PC_IP>`, `<UI_PC_IP>`, `<ROBOT_CONTROLLER_IP>`는 괄호를 포함해 그대로 입력하지 말고 실제 IP 주소로 교체합니다.

### Step 0. 실행 전 확인

- 로봇, 작업대, 약통, 스크래퍼, 종이봉투가 티칭한 좌표와 같은 위치에 있는지 확인합니다.
- Doosan Controller에 `Tool Weight`, `Tool_v1`, `Tool_scraper` 설정이 등록되어 있는지 확인합니다.
- 작업 반경을 비우고 비상 정지 버튼을 바로 사용할 수 있는 상태로 둡니다.
- 여러 PC에서 ROS 2를 사용하면 같은 `ROS_DOMAIN_ID`를 사용하고 `ROS_LOCALHOST_ONLY=0`으로 설정합니다.
- `pjt_front_ui/app.py`의 `DB_API_BASE_URL`을 실제 Backend PC 주소로 확인합니다.

최초 실행 또는 ROS 2 패키지 수정 후에는 워크스페이스를 빌드합니다.

```bash
cd ~/peterws/ws_cobot_pjt/ws_dsr
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select rokey
source install/setup.bash
```

### Step 1. Pharmacy Backend 실행

Backend PC에서 Django API 서버를 실행합니다.

```bash
cd ~/peterws/ws_cobot_pjt/ws_dsr/src/rokey/pharmacy_backend

# 최초 실행 또는 DB 스키마 변경 시
python3 manage.py migrate

python3 manage.py runserver 0.0.0.0:8000
```

다른 PC에서 API 연결을 확인할 수 있습니다.

```bash
curl http://<BACKEND_PC_IP>:8000/api/events/
curl http://<BACKEND_PC_IP>:8000/api/medicine/
```

### Step 2. Front UI 실행

UI PC에서 ROS 2 환경을 불러온 후 Flask UI를 실행합니다.

```bash
cd ~/peterws/ws_cobot_pjt/ws_dsr
source /opt/ros/humble/setup.bash
source install/setup.bash

cd src/rokey
python3 pjt_front_ui/app.py
```

웹 브라우저에서 `http://<UI_PC_IP>:5000`으로 접속합니다.

### Step 3. Doosan Robot Bringup 실행

Robot Control PC에서 M0609 컨트롤러와 ROS 2를 연결합니다.

```bash
cd ~/peterws/ws_cobot_pjt/ws_dsr
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py \
  mode:=real host:=<ROBOT_CONTROLLER_IP> port:=12345 model:=m0609
```

### Step 4. Task Manager Bridge 실행

새 터미널에서 Backend의 Event/Medicine 정보를 ROS 2 Topic으로 변환하는 브릿지를 실행합니다.

```bash
cd ~/peterws/ws_cobot_pjt/ws_dsr
source /opt/ros/humble/setup.bash
source install/setup.bash

cd src/rokey
python3 -m rokey.task_manager_bridge_final --ros-args \
  -p backend_base_url:=http://<BACKEND_PC_IP>:8000/api
```

### Step 5. 통합 로봇 제어 노드 실행

새 터미널에서 로봇 제어 노드 `final.py`를 실행합니다. `PHARMACY_DONE_URL`은 약품 리필 완료 결과를 전송할 Backend API 주소입니다.

```bash
cd ~/peterws/ws_cobot_pjt/ws_dsr
source /opt/ros/humble/setup.bash
source install/setup.bash

export PHARMACY_DONE_URL=http://<BACKEND_PC_IP>:8000/api/tasks/refill/

cd src/rokey
python3 -m rokey.final
```

> 현재 `setup.py`에 최종 노드의 `console_scripts` 등록이 없으므로 Python 모듈 형식으로 실행합니다. 추후 실행 엔트리를 등록하고 다시 빌드하면 `ros2 run rokey final`, `ros2 run rokey task_manager_bridge_final`로 실행할 수 있습니다.

### Step 6. 처방 등록 및 작업 확인

1. Front UI에서 처방전을 등록합니다.
2. `task_manager_bridge_final` 로그에서 Event/Medicine 조회 및 Topic 발행을 확인합니다.
3. `final` 노드가 리필 필요 약품을 순서대로 처리한 후 조제와 종이봉투 작업을 수행하는지 확인합니다.
4. 작업 중에는 작업 반경에 진입하지 않고 로봇 상태와 비상 정지 장치를 계속 감시합니다.

필요한 경우 다른 터미널에서 주요 Topic을 확인할 수 있습니다.

```bash
source /opt/ros/humble/setup.bash
source ~/peterws/ws_cobot_pjt/ws_dsr/install/setup.bash

ros2 topic echo /dsr01/pharmacy/refill_required_medicine
ros2 topic echo /dsr01/pharmacy/scraper_task
ros2 topic echo /dsr01/task_done
```

### 종료 순서

1. 처방 작업이 종료되고 로봇이 정지한 것을 확인합니다.
2. `final` 노드를 `Ctrl+C`로 종료합니다.
3. `task_manager_bridge_final`, Front UI, Backend를 순서대로 종료합니다.
4. 마지막으로 Doosan bringup과 로봇 컨트롤러를 종료합니다.

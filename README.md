# 약국 업무 보조 로봇 시스템

> **팀:** C-2 ROKEY
>
> **팀원:** 김혜승, 박현정, 서영채, 조해벽

Doosan Robotics M0609 협동로봇을 이용해 약품 리필, 약품 조제 보조,
스크래퍼 이송, 종이봉투 수납 작업을 수행하는 시스템입니다.

## 1. 시스템 설계 및 흐름

### 1-1. 시스템 설계도

<p align="center">
  <img width="4398" height="6322" alt="협동1 시스템 아키텍처" src="https://github.com/user-attachments/assets/7557a425-36a0-494f-8d38-b805b70bbfb5" />
</p>

[시스템 설계도 원본 보기](https://drive.google.com/file/d/1rQMJtL6ISLpyqz9C98pV-1yGLuqX5OQS/view?usp=sharing)

PC, 웹 UI, Django Backend, ROS 2 노드 및 M0609 사이의 통신 구조를
나타냅니다.

### 1-2. 전체 작업 흐름

<p align="center">
  <img width="1531" height="2031" alt="협동1 플로우차트" src="https://github.com/user-attachments/assets/9dffe982-0576-4049-88eb-5fdefcf9f6e8" />
</p>

처방 등록부터 약품 리필 여부 판단, 조제, 스크래퍼 처리 및 종이봉투
수납까지의 전체 작업 순서를 나타냅니다.

## 2. 운영체제 환경

- **OS:** Ubuntu 22.04 LTS
- **ROS 2:** Humble
- **Python:** 3.10.12
- **Robot:** Doosan Robotics M0609
- **IDE:** Visual Studio Code

## 3. 사용 장비

| 장비 | 수량 | 비고 |
|---|:---:|---|
| Doosan Robotics M0609 | 1 | 협동로봇 |
| PC | 3 | 로봇 제어, UI, Backend |
| 조제기 서랍장 | 1 | 약품 조제 공간 |
| 그리퍼 | 1 | Digital Output 기반 |
| 툴 거치대 | 1 | 3D 프린팅 |
| 스크래퍼 | 1 | 3D 프린팅 |
| 약병 오프너 | 1 | 3D 프린팅 |
| 약병 고정틀 | 1 | 3D 프린팅 |
| 약품 적재소 | 1 | 선반 |

<details>
  <summary>사용 장비 사진</summary>

  #### Doosan M0609
  <img width="2252" height="4000" alt="Doosan M0609" src="https://github.com/user-attachments/assets/6c2c5e06-3799-4459-9d04-16c5c0604831" />

  #### 그리퍼
  <img width="2252" height="4000" alt="그리퍼" src="https://github.com/user-attachments/assets/f5e704d9-a003-4fa7-8b38-fc74972df3f4" />

  #### 조제기 서랍장
  <img width="2252" height="4000" alt="조제기 서랍장" src="https://github.com/user-attachments/assets/aadff99e-1969-49fa-a882-1434a668eb19" />

  #### 툴 거치대
  <img width="2252" height="4000" alt="툴 거치대" src="https://github.com/user-attachments/assets/8b37e0db-808e-4fbd-a95c-596e435f59b3" />

  #### 스크래퍼
  <img width="2252" height="4000" alt="스크래퍼" src="https://github.com/user-attachments/assets/daa9cc56-0900-4d0b-a873-d22895193a1c" />

  #### 약병 오프너
  <img width="2252" height="4000" alt="약병 오프너" src="https://github.com/user-attachments/assets/9b92996c-56ca-43a4-9715-9b10ef94d329" />

  #### 약병 고정틀
  <img width="4000" height="2252" alt="약병 고정틀" src="https://github.com/user-attachments/assets/bc6dc640-6042-464e-9574-1cf5a15edeba" />
</details>

<p align="center">
  <img width="894" height="665" alt="전체 작업 공간 및 장비 배치" src="https://github.com/user-attachments/assets/9b5aa929-602f-4e30-9a34-23a4cf961427" />
</p>

## 4. 프로젝트 구성

```text
rokey/
├── rokey/
│   ├── __init__.py
│   ├── final.py                         # 통합 로봇 제어 노드
│   └── task_manager_bridge_final.py     # Backend GET → ROS 2 Topic 브릿지
├── pjt_front_ui/                        # Flask 및 Socket.IO 웹 UI
├── pharmacy_backend/                    # Django REST Backend
├── resource/rokey                       # ament_python 패키지 마커
├── .env.example                         # 환경변수 예시
├── package.xml
├── requirements.txt
├── setup.cfg
└── setup.py
```

### 핵심 구성요소

- **`rokey/final.py`**: 리필, 약병 개봉, 약 붓기, 스크래퍼 및
  종이봉투 작업을 하나의 FIFO 작업 큐에서 수행합니다.
- **`rokey/task_manager_bridge_final.py`**: Backend의
  `/api/events/`, `/api/medicine/`를 주기적으로 GET하고 로봇이 사용할
  JSON Topic을 발행합니다. 이 브릿지는 Backend에 POST하지 않습니다.
- **`pjt_front_ui/`**: 처방 입력, 재고 및 작업 상태를 보여주는 Flask UI입니다.
- **`pharmacy_backend/`**: 처방, 약품, Event 및 EventItem을 관리하는
  Django REST API입니다.

## 5. 의존성 및 빌드

### 5-1. ROS 2 및 Doosan Robotics

다음 ROS 2 패키지가 필요합니다.

- `rclpy`
- `std_msgs`
- `dsr_bringup2`
- `dsr_common2`
- `dsr_msgs2`

`DR_init`, `DSR_ROBOT2`, `DR_common2`는 pip 라이브러리가 아니라
`dsr_common2`를 빌드하면 제공되는 Python 모듈입니다.

### 5-2. Python 라이브러리

```bash
cd ~/peterws/ws_cobot_pjt/ws_dsr/src/rokey
python3 -m pip install -r requirements.txt
```

주요 라이브러리는 다음과 같습니다.

- `requests`
- `Flask`, `Flask-SocketIO`
- `Django`, `djangorestframework`, `django-cors-headers`
- `psycopg2-binary`

### 5-3. ROS 2 패키지 빌드

```bash
cd ~/peterws/ws_cobot_pjt/ws_dsr
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select rokey
source install/setup.bash
```

빌드 후 최종 실행 파일을 확인할 수 있습니다.

```bash
ros2 pkg executables rokey
```

예상 결과:

```text
rokey final
rokey task_manager_bridge_final
```

## 6. 환경변수

환경변수 예시는 [`.env.example`](.env.example)에 있습니다. 실제 비밀번호와
Secret Key가 들어 있는 `.env` 파일은 Git에 커밋하지 않습니다.

| 변수 | 사용 위치 | 기본값 및 역할 |
|---|---|---|
| `PHARMACY_BACKEND_URL` | Front UI | `http://127.0.0.1:8000` |
| `PHARMACY_DONE_URL` | `final` | 리필 완료 POST API |
| `PHARMACY_UI_SECRET_KEY` | Front UI | Flask 세션 Secret Key |
| `DJANGO_SECRET_KEY` | Backend | Django Secret Key |
| `DJANGO_DEBUG` | Backend | `0`; 개발 시에만 `1` |
| `DJANGO_ALLOWED_HOSTS` | Backend | 접속을 허용할 호스트 목록 |
| `DB_ENGINE` | Backend | 미설정 시 SQLite 사용 |
| `DB_NAME`, `DB_USER`, `DB_PASSWORD`, `DB_HOST`, `DB_PORT` | Backend | PostgreSQL/Supabase 접속정보 |

PostgreSQL/Supabase를 사용하지 않으면 Backend는 기본적으로
`pharmacy_backend/db.sqlite3`를 사용합니다.

> 이전 Git 이력에 포함됐던 Django 및 Supabase 자격증명은 현재 파일에서
> 제거했습니다. 이미 노출된 DB 비밀번호는 반드시 폐기하고 새 비밀번호로
> 교체해야 합니다.

## 7. 실행 순서

아래 순서는 Backend PC, UI PC, Robot Control PC를 분리해 실행하는
기준입니다. 한 PC에서 모두 실행하면 `BACKEND_PC_IP=127.0.0.1`로
설정할 수 있습니다.

### Step 0. 실행 전 점검

1. 로봇, 작업대, 약통, 스크래퍼 및 종이봉투가 티칭 좌표와 같은 위치에
   있는지 확인합니다.
2. Doosan Controller에 `Tool Weight`, `Tool_v1`, `Tool_scraper`가
   등록돼 있는지 확인합니다.
3. 작업 반경을 비우고 비상 정지 버튼을 즉시 사용할 수 있게 합니다.
4. 여러 PC의 `ROS_DOMAIN_ID`를 동일하게 설정하고
   `ROS_LOCALHOST_ONLY=0`으로 설정합니다.
5. 처음 실행하거나 코드가 변경됐다면 5절의 빌드를 다시 수행합니다.

각 PC에서 실제 환경에 맞게 IP를 설정합니다.

```bash
# 예시이므로 실제 장비 IP로 변경
export BACKEND_PC_IP=172.23.0.128
export UI_PC_IP=172.23.0.129
export ROBOT_CONTROLLER_IP=192.168.1.100
```

환경변수는 터미널마다 적용되므로 새 터미널을 열었을 때 다시 설정하거나,
각 PC의 셸 환경에서 불러와야 합니다.

### Step 1. Pharmacy Backend 실행

Backend PC에서 실행합니다. SQLite를 사용할 경우 별도의 DB 환경변수가
필요하지 않습니다.

```bash
cd ~/peterws/ws_cobot_pjt/ws_dsr/src/rokey/pharmacy_backend

export DJANGO_SECRET_KEY="replace-with-a-random-secret"
export DJANGO_ALLOWED_HOSTS="127.0.0.1,localhost,${BACKEND_PC_IP}"

python3 manage.py migrate
python3 manage.py runserver 0.0.0.0:8000
```

PostgreSQL/Supabase를 사용하는 경우 실행 전에 `.env.example`에 표시된
`DB_*` 환경변수도 설정합니다.

새 SQLite DB에는 약품 정보가 들어 있지 않습니다. 실제 처방 테스트 전에는
관리자 페이지나 API를 통해 Medicine 데이터를 먼저 등록하거나, 기존
PostgreSQL/Supabase DB에 연결해야 합니다.

다른 PC에서 API 연결을 확인합니다.

```bash
curl "http://${BACKEND_PC_IP}:8000/api/events/"
curl "http://${BACKEND_PC_IP}:8000/api/medicine/"
```

### Step 2. Front UI 실행

UI PC의 새 터미널에서 실행합니다.

```bash
cd ~/peterws/ws_cobot_pjt/ws_dsr
source /opt/ros/humble/setup.bash
source install/setup.bash

export PHARMACY_BACKEND_URL="http://${BACKEND_PC_IP}:8000"
export PHARMACY_UI_SECRET_KEY="replace-with-a-random-secret"

cd src/rokey
python3 pjt_front_ui/app.py
```

브라우저에서 `http://<UI_PC_IP>:5000`으로 접속합니다. 여기서
`<UI_PC_IP>`는 실제 UI PC 주소로 바꿉니다.

### Step 3. Doosan Robot Bringup 실행

Robot Control PC에서 M0609 컨트롤러와 ROS 2를 연결합니다.

```bash
cd ~/peterws/ws_cobot_pjt/ws_dsr
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py \
  mode:=real \
  host:=${ROBOT_CONTROLLER_IP} \
  port:=12345 \
  model:=m0609
```

### Step 4. Task Manager Bridge 실행

Robot Control PC의 새 터미널에서 Backend 정보를 ROS 2 Topic으로 변환하는
브릿지를 실행합니다.

```bash
cd ~/peterws/ws_cobot_pjt/ws_dsr
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run rokey task_manager_bridge_final --ros-args \
  -p backend_base_url:="http://${BACKEND_PC_IP}:8000/api"
```

### Step 5. 통합 로봇 제어 노드 실행

Robot Control PC의 새 터미널에서 실행합니다.

```bash
cd ~/peterws/ws_cobot_pjt/ws_dsr
source /opt/ros/humble/setup.bash
source install/setup.bash

export PHARMACY_DONE_URL="http://${BACKEND_PC_IP}:8000/api/tasks/refill/"
ros2 run rokey final
```

### Step 6. 처방 등록 및 작업 확인

1. Front UI에서 처방전을 등록합니다.
2. `task_manager_bridge_final`이 Event와 Medicine을 조회하고 작업 Topic을
   발행하는지 확인합니다.
3. `final`이 리필 작업과 조제·스크래퍼·종이봉투 작업을 순서대로
   수행하는지 확인합니다.
4. 작업 중에는 로봇 작업 반경에 진입하지 않습니다.

주요 Topic은 별도 터미널에서 확인할 수 있습니다.

```bash
source /opt/ros/humble/setup.bash
source ~/peterws/ws_cobot_pjt/ws_dsr/install/setup.bash

ros2 topic echo /dsr01/pharmacy/events
ros2 topic echo /dsr01/pharmacy/medicine
ros2 topic echo /dsr01/pharmacy/refill_required_medicine
ros2 topic echo /dsr01/pharmacy/scraper_task
ros2 topic echo /dsr01/task_done
```

### 종료 순서

1. 현재 작업이 끝나고 로봇이 정지한 것을 확인합니다.
2. `final`을 `Ctrl+C`로 종료합니다.
3. `task_manager_bridge_final`, Front UI, Backend를 종료합니다.
4. 마지막으로 Doosan bringup과 로봇 컨트롤러를 종료합니다.

## 8. ROS 2 Topic

| Topic | 형식 | Publisher | Subscriber | 역할 |
|---|---|---|---|---|
| `/dsr01/pharmacy/events` | `std_msgs/String` JSON | `task_manager_bridge_final` | 모니터링 노드 | Backend Event 조회 결과 |
| `/dsr01/pharmacy/medicine` | `std_msgs/String` JSON | `task_manager_bridge_final` | 모니터링 노드 | Backend Medicine 조회 결과 |
| `/dsr01/pharmacy/refill_required_medicine` | `std_msgs/String` JSON | `task_manager_bridge_final` | `final` | 리필이 필요한 약품과 좌표 |
| `/dsr01/pharmacy/scraper_task` | `std_msgs/String` JSON | `task_manager_bridge_final` | `final` | 모든 약품 준비 후 조제 작업 명령 |
| `/dsr01/task_done` | `std_msgs/Bool` | `final` | 완료 처리 및 모니터링 노드 | 종이봉투 수납 완료 |

## 9. 안전 및 운영 주의사항

- 본 코드는 실제 로봇을 움직이므로 처음에는 낮은 속도에서 단일 단계로
  검증합니다.
- 티칭 좌표와 Tool/TCP 설정이 달라지면 즉시 동작을 중지하고 다시
  검증합니다.
- 힘 제어가 종료되는 모든 경로에서 Force 및 Compliance 해제가 수행되는지
  확인합니다.
- Backend 또는 ROS 2 연결이 끊기면 새 작업을 시작하지 말고 현재 로봇
  상태를 먼저 확인합니다.
- 비상 정지와 두산 컨트롤러의 안전 기능을 애플리케이션 예외처리보다
  우선합니다.

## 10. License

This project is licensed under the Apache License 2.0.

# 💊 약국 업무 보조 로봇 (Pharmacy Assistant Robot) -수정중-
> **조 이름: C-2** > **팀원: 김혜승_서영채_박현정_조해벽**

## 1. 🎨 시스템 설계 및 플로우 차트
프로젝트의 전체적인 구조와 소프트웨어 흐름도입니다.

### 1-1. 시스템 설계도 (System Architecture)
<p align="center">
  <img src="./images/system_design.png" alt="시스템 설계도 이미지" width="400">
</p>
* *설명: PC와 두산 M0609 매니퓰레이터 간의 ROS2 통신 구조 및 웹/DB 서버 아키텍처를 나타냅니다.*

### 1-2. 플로우 차트 (Flow Chart)
<p align="center">
  <img src="./images/flow_chart.png" alt="플로우 차트 이미지" width="300" height="300">
</p>
* *설명: UI 기반의 처방전 입력부터 조제기 내 의약품 사전 보충(Refill), 최종 패키징(Packaging)까지의 전체 프로세스 진행도를 나타냅니다.*

---

## 2. 🖥️ 운영체제 환경 (OS Environment)
이 프로젝트는 다음 환경에서 개발하였습니다.

* **OS:** Ubuntu 22.04 LTS
* **ROS Version:** ROS2 Humble
* **Language:** Python 3.10
* **IDE:** VS Code

---

## 3. 🛠️ 사용 장비 목록 (Hardware List)
프로젝트에 사용된 주요 하드웨어 장비입니다.

| 장비명 (Model) | 수량 | 비고 |
|:---:|:---:|:---|
| Robot (Doosan M0609) | 1 | [cite_start]협동로봇 매니퓰레이터 [cite: 489, 504] |
| 그리퍼툴 | 1 | [cite_start]뚜껑 개봉 및 약봉투 파지용 [cite: 489] |
| 조제기 서랍 및 약통 | 1 | [cite_start]사전 보충 테스트용 [cite: 489] |
| 약 종이 봉투 고정대 | 1 | [cite_start]패키징 작업 지그 [cite: 489] |
| 투명 약 봉투 거치대 | 1 | [cite_start]투명 약봉지 픽업용 [cite: 489] |

---

## 4. 📦 의존성 (Dependencies)
프로젝트 실행에 필요한 라이브러리입니다.

* Python >= 3.10
* rclpy (ROS2 Python Client)
* [cite_start]FastAPI [cite: 472]
* Flask [cite: 470]
* [cite_start]SQLite [cite: 471]

---

## 5. ▶️ 실행 순서 (Usage Guide)
프로젝트를 실행하기 위한 순서입니다. 터미널 명령어를 순서대로 입력해 주세요.

### Step 1. 로봇 초기화
로봇의 전원을 켜고 ROS2 드라이버 통신을 연결합니다.
```bash
[cite_start]ros2 launch m0609_rg2_bringup bringup.launch.py mode:=real host:=192.168.1.100 port:=12345 model:=m0609 [cite: 24]

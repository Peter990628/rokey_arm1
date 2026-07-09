# [프로젝트 이름] (예: 커피 제조 자동화 시스템)
> **조 이름:** [A-1 - ROKEY]
> **팀원:** [홍길동_김두산_이로키_박캠프]

## 1. 🎨 시스템 설계 및 플로우 차트
프로젝트의 전체적인 구조와 소프트웨어 흐름도입니다.

### 1-1. 시스템 설계도 (System Architecture)
<p align="center">
  <img src="./images/system_design.png" alt="시스템 설계도 이미지" width="400">
</p>
* *설명: [예: PC와 매니퓰레이터 간의 통신 구조를 나타냅니다.]*

### 1-2. 플로우 차트 (Flow Chart)
<p align="center">
  <img src="./images/flow_chart.png" alt="플로우 차트 이미지" width="300" height="300">
</p>
* *설명: [예: UI부터 전체 프로세스 진행도를 나타냅니다.]*

---

## 2. 🖥️ 운영체제 환경 (OS Environment)
이 프로젝트는 다음 환경에서 개발하였습니다.

* **OS:** [예: Ubuntu 22.04 LTS]
* **ROS Version:** [예: ROS2 Humble]
* **Language:** [예: Python 3.8 ]
* **IDE:** [예: VS Code]

---

## 3. 🛠️ 사용 장비 목록 (Hardware List)
프로젝트에 사용된 주요 하드웨어 장비입니다.

| 장비명 (Model) | 수량 | 비고 |
|:---:|:---:|:---|
| [예: Arduino Uno] | 1 | [ ] |
| [예: 컨베이어 벨트] | 1 | [ ] |
| [예: 초음파 센서] | 1 | [ ] |

---

## 4. 📦 의존성 (Dependencies)
프로젝트 실행에 필요한 라이브러리입니다.

* **[예시]**
* Python >= 3.8
* firebase_admin 6.6.0



---

## 5. ▶️ 실행 순서 (Usage Guide)
프로젝트를 실행하기 위한 순서입니다. 터미널 명령어를 순서대로 입력해 주세요.
### Step 1. [예: 로봇 초기화] 로봇의 전원을 켜고 통신을 연결합니다.
```bash
ros2 launch  dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=real host:=192.168.1.100 port:=12345 model:=m0609
```
### Step 2. [예: 메인 제어 노드 실행] 기능 알고리즘을 시작합니다.
```bash
ros2 run my_project main_control.py
```

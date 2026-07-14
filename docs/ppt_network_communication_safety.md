# PPT 시스템 설계 자료

작성 기준: 2026-07-14 현재 `robot_total.py`, `task_manager_bridge.py`, `pharmacy_backend`, `pjt_front_ui` 코드

> 이 문서는 발표 및 프로젝트 설계 검토용이다. 안전 정격 설계서나 인증 문서를 대신하지 않는다. 실제 속도, 힘, 안전 영역, 정지 방식은 설치된 치공구와 작업 환경을 대상으로 위험성 평가 및 정지거리 검증 후 확정해야 한다.

## 권장 슬라이드 구성

1. 네트워크 구성도
2. ROS 2 토픽 통신 정의
3. HTTP 및 Doosan 서비스 통신 정의
4. 오류·예외 예상 목록 및 처리 방안
5. 위험요소 및 안전대책
6. 현재 통합 이슈와 개선 우선순위

---

## 1. 네트워크 구성도

PPT에는 [network_configuration.svg](./network_configuration.svg)를 삽입하는 것을 권장한다. SVG는 확대해도 선명하며, PNG가 필요한 경우 [network_configuration.png](./network_configuration.png)를 사용한다.

### 구성 요소

| 구역 | 장치/프로세스 | 역할 | 주요 통신 |
|---|---|---|---|
| 운영자 PC | Browser, Flask, Socket.IO, `ui_alarm_node` | 처방 입력, 재고·작업 상태 표시 | HTTP/WebSocket `:5000`, ROS 2 DDS |
| Backend PC | Django REST API | 처방, 약품, 이벤트, 재고 관리 | HTTP `:8000`, PostgreSQL `:5432` |
| Robot PC | `task_manager_bridge` | Backend의 Event/Medicine을 읽어 ROS 토픽으로 변환 | HTTP GET → ROS 2 Topic |
| Robot PC | `robot_total` | 작업 큐, 개봉·리필·포장 순차 제어 | ROS 2 Topic, HTTP POST, ROS 2 Service |
| Robot PC | `DSR_ROBOT2`, `dsr_controller2` | Python 명령을 Doosan ROS 서비스 및 Controller 명령으로 변환 | ROS 2 Service, Controller TCP |
| 조제기/HMI | 조제 설비 제어 노드 | 스크래퍼 준비/조제 완료 핸드셰이크 | ROS 2 Bool Topic |
| Doosan Controller | M0609 Controller | 모션, 힘 제어, I/O, 안전 기능 실행 | 기본 TCP `:12345` |
| Cloud DB | Supabase PostgreSQL | Medicine, Event, EventItem 영속 저장 | PostgreSQL/TCP `:5432` |

### 현재 확인된 네트워크 설정

| 항목 | 현재 값 | 비고 |
|---|---:|---|
| Backend 주소 | `172.23.0.128:8000` | 코드 기본값이며 실행 시 ROS 파라미터로 변경 가능 |
| ROS Domain | `ROS_DOMAIN_ID=26` | ROS 통신 PC 모두 동일해야 함 |
| Localhost 제한 | `ROS_LOCALHOST_ONLY=0` | 다른 PC 검색 허용 |
| RMW | 명시하지 않음 | 모든 PC에서 같은 구현을 명시하는 것을 권장 |
| Robot namespace | `/dsr01` | `robot_total` 및 Doosan 서비스 이름에 반영 |
| Controller 주소 | 제조사 기본 `192.168.137.100:12345` | 실제 Controller/PC NIC 설정을 현장에서 확인 |

### 네트워크 점검 항목

- Robot PC에서 `curl http://172.23.0.128:8000/api/events/`와 `/api/medicine/` 응답 확인
- ROS PC 모두 `ROS_DOMAIN_ID=26`, `ROS_LOCALHOST_ONLY=0`, 동일 RMW 사용
- Wi-Fi/AP 및 방화벽에서 DDS 멀티캐스트와 UDP 동적 포트 허용
- `ros2 multicast send` / `ros2 multicast receive`로 PC 간 멀티캐스트 확인
- `ros2 node list`, `ros2 topic list`, `ros2 service list`로 DDS Discovery 확인
- Controller 전용 NIC와 업무망 NIC를 구분하고 라우팅 우선순위 확인
- Backend/DB 접근은 필요한 포트와 호스트로 제한

---

## 2. ROS 2 토픽 통신 정의서

모든 토픽은 코드에서 depth `10`으로 생성된다. History/신뢰성 등 나머지 QoS는 현재 `rclpy` 기본값을 사용한다.

### 통합 제어 토픽

| 토픽 | 형식 | Publisher | Subscriber | 발행 조건 및 역할 |
|---|---|---|---|---|
| `/dsr01/pharmacy/events` | `std_msgs/String` JSON | `task_manager_bridge` | `robot_total` | `/api/events/` 응답 전체를 1초마다 발행 |
| `/dsr01/pharmacy/medicine` | `std_msgs/String` JSON | `task_manager_bridge` | `robot_total` | `/api/medicine/` 응답 전체를 1초마다 발행 |
| `/dsr01/pharmacy/refill_required_medicine` | `std_msgs/String` JSON | `task_manager_bridge` | `robot_total` | EventItem 중 `READY`가 아닌 약의 위치·offset 정보를 발행. 대상이 없으면 발행하지 않음 |
| `/dsr01/pharmacy/robot_enable` | `std_msgs/Bool` | 운영/HMI 노드 | `robot_total` | `true`: 작업 시작 허가, `false`: 소프트웨어 작업 비활성화 |
| `/dsr01/pharmacy/scraper_ready` | `std_msgs/Bool` | `robot_total` | 조제기/HMI | 스크래퍼가 배출구에 도착하면 `true`, 대기 종료 시 `false` |
| `/dsr01/pharmacy/dispensing_done` | `std_msgs/Bool` | 조제기/HMI | `robot_total` | 조제기의 약 배출 완료 시 `true` |
| `/dsr01/pharmacy/robot_status` | `std_msgs/String` JSON | `robot_total` | 운영/HMI 노드 | 통합 노드 상태, 상세 설명, enable 상태 발행 |
| `/dsr01/task_done` | `std_msgs/Bool` | `robot_total` 내부 `ManipulatorTest2` | 완료 처리 노드 | 종이봉투 수납 완료 후 `true`; 상대 이름 `task_done`이 `/dsr01` namespace에서 해석된 결과 |

### 주요 JSON 형식

`/dsr01/pharmacy/events`

```json
[
  {
    "id": 1,
    "prescription_name": "홍길동",
    "status": "WAITING",
    "created_at": "2026-07-14T00:00:00Z",
    "items": [
      {
        "id": 10,
        "medicine_name": "타이레놀",
        "quantity": 2,
        "order": 1,
        "status": "READY"
      }
    ]
  }
]
```

`/dsr01/pharmacy/refill_required_medicine`

```json
[
  {
    "medicine_name": "타이레놀",
    "storage_pose": {"x": 0.0, "y": 0.0, "z": 0.0, "rx": 0.0, "ry": 0.0, "rz": 0.0},
    "dispensing_pose": {"x": 0.0, "y": 0.0, "z": 0.0, "rx": 0.0, "ry": 0.0, "rz": 0.0},
    "drawer_pose": {"x": 0.0, "y": 0.0, "z": 0.0, "rx": 0.0, "ry": 0.0, "rz": 0.0},
    "bottle_tip_offset_x": 0.0,
    "bottle_tip_offset_y": 25.0,
    "bottle_tip_offset_z": -42.0
  }
]
```

`/dsr01/pharmacy/robot_status`

```json
{
  "state": "REFILLING",
  "detail": "타이레놀 조제기 리필",
  "enabled": true
}
```

상태값은 `DISABLED`, `IDLE`, `OPENING`, `REFILLING`, `SCRAPER_PICKUP`, `WAITING_DISPENSING`, `PACKAGING`, `ERROR`이다.

### 현재 UI 토픽 불일치

`pjt_front_ui/app.py`는 아래 토픽을 `String(JSON)`으로 구독하지만, 현재 `robot_total`은 같은 이름과 형식으로 발행하지 않는다.

| Front UI가 기다리는 토픽 | 현재 `robot_total` 대응 | 판정 |
|---|---|---|
| `/pharmacy/task_start` `String` | `/dsr01/pharmacy/robot_status` `String` | 이름/스키마 매핑 필요 |
| `/pharmacy/task_done` `String` | `/dsr01/task_done` `Bool` | 이름과 메시지 형식 모두 불일치 |
| `/pharmacy/remaining_count` `String` | 직접 대응 토픽 없음 | Backend 조회 또는 신규 발행 필요 |

권장안은 UI가 `/dsr01/pharmacy/robot_status`를 직접 구독하고, 완료 메시지를 `String(JSON)` 한 종류로 통일하는 것이다. 완료 JSON에는 최소 `event_id`, `state`, `detail`, `success`를 포함한다.

---

## 3. HTTP 통신 정의서

| Method / Endpoint | 호출자 | 목적 | 주요 데이터 | 현재 timeout |
|---|---|---|---|---:|
| `GET /api/events/` | Front UI, `task_manager_bridge` | 처방 이벤트 조회 | Event 및 EventItem 목록 | UI 5초, Bridge 2초 |
| `GET /api/medicine/` | Front UI, `task_manager_bridge` | 약품·좌표·재고 조회 | Medicine 목록 | UI 5초, Bridge 2초 |
| `POST /api/prescriptions/` | Front UI | 처방 등록 | `prescription_name`, `items[]` | 5초 |
| `POST /api/tasks/status/` | Front UI | 검수/이벤트 상태 변경 | `event_id`, `status` | 5초 |
| `POST /api/tasks/refill/` | `robot_total` | 물리적 리필 완료 후 DB 재고 갱신 | `medicine_name`, `amount` | 2초 |

`task_manager_bridge`는 읽기 전용이며 `GET /api/events/`, `GET /api/medicine/`만 수행한다. POST는 하지 않는다.

---

## 4. Doosan ROS 2 서비스 통신 정의서

공통 구조는 `robot_total → DSR_ROBOT2(Service Client) → dsr_controller2(Service Server) → Doosan Controller`이다. 아래 이름은 `/dsr01` namespace가 적용된 실제 예상 이름이다.

| 서비스 이름 | 서비스 형식 | 호출 함수 | 용도 |
|---|---|---|---|
| `/dsr01/motion/move_joint` | `dsr_msgs2/srv/MoveJoint` | `movej()` | 관절 좌표 이동 |
| `/dsr01/motion/move_jointx` | `dsr_msgs2/srv/MoveJointx` | `movejx()` | Task pose를 관절 보간으로 이동 |
| `/dsr01/motion/move_line` | `dsr_msgs2/srv/MoveLine` | `movel()`, `amovel()` | 직선 이동 및 비동기 직선 이동 |
| `/dsr01/motion/move_periodic` | `dsr_msgs2/srv/MovePeriodic` | `amove_periodic()` | 약통 붓기 등의 주기 운동 |
| `/dsr01/motion/trans` | `dsr_msgs2/srv/Trans` | `trans()` | 좌표계 변환 |
| `/dsr01/aux_control/get_current_posx` | `dsr_msgs2/srv/GetCurrentPosx` | `get_current_posx()` | 현재 TCP pose 조회 |
| `/dsr01/aux_control/get_current_posj` | `dsr_msgs2/srv/GetCurrentPosj` | `get_current_posj()` | 현재 관절각 조회 |
| `/dsr01/aux_control/get_tool_force` | `dsr_msgs2/srv/GetToolForce` | `get_tool_force()` | TCP 외력 조회 |
| `/dsr01/force/task_compliance_ctrl` | `dsr_msgs2/srv/TaskComplianceCtrl` | `task_compliance_ctrl()` | Task compliance 시작 및 강성 설정 |
| `/dsr01/force/set_desired_force` | `dsr_msgs2/srv/SetDesiredForce` | `set_desired_force()` | 목표 힘과 힘 방향 설정 |
| `/dsr01/force/release_force` | `dsr_msgs2/srv/ReleaseForce` | `release_force()` | 힘 명령 해제 |
| `/dsr01/force/release_compliance_ctrl` | `dsr_msgs2/srv/ReleaseComplianceCtrl` | `release_compliance_ctrl()` | Compliance 모드 해제 |
| `/dsr01/force/check_position_condition` | `dsr_msgs2/srv/CheckPositionCondition` | `check_position_condition()` | 위치 조건 판정 |
| `/dsr01/io/set_ctrl_box_digital_output` | `dsr_msgs2/srv/SetCtrlBoxDigitalOutput` | `set_digital_output()` | 그리퍼 열기/닫기 출력 |
| `/dsr01/io/get_ctrl_box_digital_input` | `dsr_msgs2/srv/GetCtrlBoxDigitalInput` | `get_digital_input()` | 약통 파지 센서 입력 확인 |
| `/dsr01/tcp/set_current_tcp` | `dsr_msgs2/srv/SetCurrentTcp` | `set_tcp()` | 그리퍼/스크래퍼 TCP 전환 |
| `/dsr01/tool/set_current_tool` | `dsr_msgs2/srv/SetCurrentTool` | `set_tool()` | Tool weight 설정 전환 |

실행 중인 시스템에서는 다음 명령으로 최종 이름과 형식을 확정한다.

```bash
ros2 service list -t | rg '^/dsr01/'
ros2 service type /dsr01/motion/move_joint
ros2 interface show dsr_msgs2/srv/MoveJoint
```

---

## 5. 오류·예외 예상 목록 및 처리 방안

### 통신·데이터 오류

| 우선도 | 예상 오류 | 현재 감지/처리 | 권장 처리 방안 |
|---|---|---|---|
| 높음 | Backend 연결 실패/timeout | Bridge는 발행하지 않고 경고, Robot POST 실패 시 `ERROR` 및 disable | Heartbeat와 데이터 timestamp 추가, 일정 시간 이상 stale이면 신규 작업 금지 |
| 높음 | 물리 리필은 끝났지만 DB POST 실패 | `ERROR`로 전환되나 물리 재고와 DB가 불일치 | idempotency key 사용, 자동 재동작 금지, 운영자 재고 대조 후 DB만 보정 |
| 높음 | 잘못된 좌표·TCP·Tool 데이터 | 일부 필수 필드와 봉투 수납대 범위만 검사 | 모든 pose의 숫자/범위/작업영역 검사, TCP/Tool 이름 조회, 저속 dry-run |
| 높음 | 리필량 과다 계산 | 현재 `storage_stock` 전체를 `refill_amount`로 사용 | 조제기 용량과 부족량으로 `refill_amount`를 Backend에서 명시해 전달 |
| 중간 | JSON 파싱 실패 | 오류 로그 후 해당 메시지 무시 | JSON Schema와 `schema_version`, 필수 키·타입 검증 추가 |
| 중간 | `medicine_name` 중복/불일치 | 이름 dictionary 매칭, 미검색 시 경고 | DB에서 이름 unique 제약 또는 `medicine_number`로 연결 |
| 중간 | Bridge 재시작 또는 Robot 재시작 후 중복 작업 | 메모리 set으로만 중복 방지 | EventItem ID 기반 영속 checkpoint와 idempotent 작업 처리 |
| 중간 | 리필 대상이 0개일 때 토픽이 오지 않음 | 아무 메시지도 발행하지 않음 | 빈 배열 `[]` 또는 별도 heartbeat를 발행해 “0개”와 “통신 단절” 구분 |
| 중간 | Front UI 상태 미반영 | 토픽 이름/형식 불일치 | 단일 상태 스키마로 통일하고 통합 테스트 추가 |
| 중간 | Backend 보안 설정/비밀정보 노출 | 현재 설정 파일에 DB 인증정보, `DEBUG=True`, 광범위 host 허용 | 노출된 인증정보 즉시 교체, 환경변수/Secret 사용, DEBUG off, host/CORS/auth 제한 |

### 로봇 동작 오류

| 우선도 | 예상 오류 | 현재 감지/처리 | 권장 처리 방안 |
|---|---|---|---|
| 높음 | `dsr_controller2` 또는 Controller 연결 실패 | 명령 예외가 상위로 전파되어 로그 후 노드 종료/작업 `ERROR` | 시작 전 필수 서비스 대기, Robot state/servo 상태 확인, Ready 전 동작 금지 |
| 높음 | 약통 파지 실패 | DI 입력 timeout, 최대 3회 재시도 후 `GraspError` | 재시도 사이 안전 retreat, 센서 stuck 진단, 실패 시 물체 상태 확인 후 수동 복구 |
| 높음 | 스크래퍼/종이봉투 파지 실패 | 별도 파지 센서 확인 없음 | 그리퍼 DI, 힘 변화, 비전 중 하나로 파지 확인 후 이동 |
| 높음 | 힘 제어가 목표 거리에 도달하지 못함 | 통합 코드에서 기본 15초 timeout, `finally`에서 force/compliance 해제 | 시간뿐 아니라 최대 이동거리·최대 힘·속도 조건을 동시에 제한 |
| 높음 | 사람/설비 충돌 | Robot Controller의 안전 정지에 의존 | E-stop/Protective Stop, 안전영역, 속도·힘·모멘텀 제한을 위험성 평가로 설정 |
| 중간 | SPIN 뚜껑 분리 실패 | 최대 5회 후 예외 | 재시도 횟수 외 torque/force 추세 판정, 뚜껑 상태를 확인한 뒤 수동 분기 |
| 중간 | PULL 뚜껑 접촉 미검출 | 제한 횟수 후 예외 | 탐색 거리 상한과 접촉 force 범위 기록, 치공구 위치 교정 안내 |
| 중간 | 조제 완료 신호 미수신 | 기본 120초 후 timeout 및 `ERROR` | 조제기 heartbeat, 취소/배출 상태 코드, operator retry 절차 추가 |
| 중간 | 봉투 수납대 위치 이탈 | 봉투를 놓지 않고 `False`, 상위에서 `ERROR` | 안전 retreat 위치를 정의하고 “물체 파지 중” 복구 절차 표시 |
| 중간 | 실행 중 `robot_enable=False` | 조제 대기 루프에서는 중단, 다른 모션 중 즉시 안전 정지를 보장하지 않음 | `robot_enable`은 작업 허가로만 사용하고 정지는 안전 I/O/Protective Stop으로 구현 |
| 중간 | 프로세스/전원 중단 | 작업 상태와 파지 상태가 불명확, 메모리 queue 손실 | 자동 재개 금지, Controller/그리퍼/작업물 점검 후 명시적 recovery 단계 수행 |

### 권장 공통 오류 처리 순서

1. 이상 감지 또는 timeout
2. 가능한 경우 비동기 이동 정지 및 force/compliance 해제
3. 현재 물체 파지 여부와 Robot state 기록
4. `/dsr01/pharmacy/robot_status`에 `ERROR`와 원인·event ID·medicine_name 발행
5. `robot_enabled=False`로 신규 작업 차단
6. 운영자가 작업 공간, 치공구, 약통, 그리퍼를 확인
7. 안전한 복구 자세로 이동한 뒤 명시적 reset/enable
8. 물리 재고와 DB를 대조하고 재시도 또는 작업 폐기 결정

자동 재시도는 “물리 상태가 변하지 않았음”이 확인되는 통신 오류에만 제한하는 것이 좋다.

---

## 6. 위험요소 및 안전대책

| 위험요소 | 예상 결과 | 공학적 대책 | 운영·소프트웨어 대책 |
|---|---|---|---|
| 사람과 로봇 충돌 | 타박상, 끼임 | 접근 가능한 E-stop, Protective Stop 입력, 안전 스캐너/인터록, 안전 속도·힘·모멘텀 제한 | 자동 운전 전 작업영역 확인, 저속 시운전, 충돌 후 원인 확인 전 reset 금지 |
| 로봇과 오프너/서랍 사이 끼임 | 손가락·팔 압착 | pinch point 간격 확보, 고정 가드, 안전영역 및 정지 모드 설정 | 개봉·서랍 동작 중 작업영역 출입 금지, 수동 제거 시 lockout/tagout |
| 약통·스크래퍼 낙하 | 설비 손상, 약품 비산 | 파지 센서, 낙하 방지 tray, 그리퍼 상태 감시 | 파지 확인 후 이동, 실패 시 자동 반복 대신 operator 확인 |
| 잘못된 TCP/Tool/payload | 경로 오차, 충돌, 힘 오판 | Tool/TCP를 Controller에 검증 등록, payload와 CoG 교정 | 작업 시작 전 프로필 확인, 프로필 전환 로그 및 pose sanity check |
| 과도한 힘 제어 | 약통 파손, 치공구 손상, 끼임 | Controller force/TCP force limit, 최대 변위·속도 제한, compliance 종료 보장 | 작은 값부터 단계 시험, force 로그 저장, timeout 후 자동 재시도 금지 |
| 잘못된 약/수량 | 오조제, 환자 위해 | 바코드/비전으로 약 ID 확인, 조제기 용량 센서, 이중 확인 장치 | `medicine_number` 기반 추적, 처방-투입-재고 reconciliation, 검수 절차 |
| 약품 유출·교차오염 | 오염, 수량 오차 | 세척 가능한 scraper/tray, 약별 분리 용기, 비산 방지 구조 | 약 종류 전환 시 청소 확인, 유출 시 해당 작업 격리 및 재고 보정 |
| 전원·네트워크 복구 후 예기치 않은 시작 | 충돌, 중복 투입 | 안전 회로 reset과 작업 시작 회로 분리 | `auto_start=False`, 자동 resume 금지, 운영자 명시적 enable 필요 |
| DB/네트워크 침해 | 잘못된 좌표·처방·재고 주입 | 방화벽, TLS, 인증/권한, 업무망과 로봇망 분리 | Secret 코드 저장 금지, 입력 검증, 감사 로그, 계정·키 주기적 교체 |

### 반드시 구분할 것

- `robot_enable` 토픽, Python 예외 처리, timeout은 **작업 제어 기능**이며 안전 정격 정지 기능이 아니다.
- 사람 보호는 Controller의 E-stop, Protective Stop, Safety I/O 및 검증된 안전 파라미터로 구성해야 한다.
- 충돌 감지, TCP Force Limit, 속도/모멘텀 제한, 안전영역과 정지 모드는 전체 workcell 위험성 평가 결과에 따라 설정해야 한다.
- 안전장치가 작동한 뒤에는 원인을 제거하고 Robot/Tool/작업물을 점검한 후 훈련된 작업자가 reset해야 한다.

### 작업 전 점검 체크리스트

- E-stop과 Protective Stop 동작 시험 완료
- Controller safety alarm 없음
- Tool weight, TCP, 그리퍼 DI/DO 확인
- 작업영역에 사람·장애물 없음
- 약통/오프너/스크래퍼/봉투 거치 위치 고정 확인
- Backend 연결, Event/Medicine 최신 시각 확인
- 저속으로 주요 접근 pose 확인
- 비상 정지 후 복구 담당자와 절차 공유

---

## 7. 현재 개선 우선순위

1. **즉시:** 저장소에 노출된 DB 인증정보 교체 및 환경변수화
2. **즉시:** Front UI와 `robot_total`의 상태/완료 토픽 이름과 메시지 형식 통일
3. **높음:** `storage_stock` 전체가 아닌 실제 부족량/조제기 용량으로 리필량 결정
4. **높음:** Backend/Topic heartbeat와 데이터 유효시간을 추가해 stale 데이터 작업 방지
5. **높음:** 스크래퍼·종이봉투 파지 성공 확인 추가
6. **높음:** 모든 좌표, TCP, Tool, payload에 대한 시작 전 검증
7. **중간:** 작업 상태와 중복 방지 정보를 DB 또는 파일에 영속화
8. **중간:** 안전 정지 후 자동 재개가 아닌 명시적 recovery 상태 구현

---

## 발표용 짧은 문구

**네트워크 구성**

> 처방·재고 정보는 HTTP REST로 전달하고, 실시간 작업 상태와 설비 핸드셰이크는 ROS 2 DDS로 교환하며, 최종 로봇 명령은 Doosan ROS 2 Service를 통해 Controller로 전달한다.

**오류 처리 정책**

> timeout 또는 동작 실패가 발생하면 force/compliance를 해제하고 신규 작업을 차단한 뒤 ERROR 상태와 원인을 발행한다. 물리 상태 확인 없이 자동 재시작하지 않는다.

**안전 설계 원칙**

> 소프트웨어 enable과 예외 처리는 작업 제어용이며, 사람 보호는 위험성 평가에 기반한 E-stop, Protective Stop, Safety I/O, 안전영역 및 속도·힘 제한으로 구현한다.

---

## 근거 자료

- 로컬 코드: `rokey/robot_total.py`, `rokey/task_manager_bridge.py`, `rokey/manipulator_test_2.py`
- 로컬 Backend/UI: `pharmacy_backend/dispensing`, `pjt_front_ui/app.py`
- [Doosan Robotics ROS 2 공식 저장소](https://github.com/DoosanRobotics/doosan-robot2)
- [Doosan Robotics Safety Function](https://manual.doosanrobotics.com/en/user-manual/3.2.2/1-m-h-series/safety-function)
- [Doosan Robotics M0609 제품 정보](https://www.doosanrobotics.com/en/product-solutions/product/m-series/m0609/)
- [ROS 2 Humble 환경 설정](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html)
- [ROS 2 Humble 멀티캐스트 점검](https://docs.ros.org/en/humble/How-To-Guides/Installation-Troubleshooting.html#enable-multicast)

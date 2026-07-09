## Pharmacy Backend 시스템 요구사항

### 1. 목적
pharmacy_backend는 약국 보조 로봇 시스템에서 처방전, 약품 정보, 재고 상태, 리필 필요 여부를 관리하는 Django REST API 서버이다.  
ROS2 브릿지 노드는 이 백엔드에서 필요한 정보를 GET으로 조회하여 로봇 제어 노드에게 전달한다.

### 2. 주요 데이터
- Medicine
  - 약 이름
  - 적재소 위치
  - 조제기 위치
  - 뚜껑 타입
  - 적재소 재고
  - 조제기 재고

- Event
  - 처방전 ID
  - 처방전 이름 / 환자 이름
  - 처방전 전체 상태
  - 생성 시간

- EventItem
  - 처방전에 포함된 약 이름
  - 필요한 약 개수
  - 처방 순서
  - 약별 상태

### 3. 상태 정의
- Event.status
  - REFILL_REQUIRED: 처방전 내 일부 약품 리필 필요
  - WAITING: 제조 가능, 작업 대기
  - PROCESSING: 제조 중
  - DONE: 제조 완료

- EventItem.status
  - READY: 조제기 재고로 처리 가능
  - REFILL_REQUIRED: 조제기 재고 부족으로 리필 필요

### 4. 주요 API
- GET `/api/medicine/`
  - 약품 목록, 위치, 재고 정보 조회

- GET `/api/events/`
  - 전체 처방전 및 제조 상태 조회

- POST `/api/prescriptions/`
  - 처방전 등록
  - 약품별 필요 개수와 조제기 재고를 비교하여 READY 또는 REFILL_REQUIRED 판단

- GET `/api/tasks/next/`
  - WAITING 상태의 다음 제조 작업 조회

- POST `/api/tasks/status/`
  - 처방전 상태 변경

- POST `/api/tasks/refill/`
  - 리필 완료 후 적재소/조제기 재고 갱신

### 5. ROS2 Bridge 연동 요구사항
- task_manager_bridge는 DB를 직접 수정하지 않는다.
- task_manager_bridge는 백엔드에 GET 요청만 수행한다.
- 조회한 JSON 데이터를 ROS2 String 토픽으로 publish한다.
- publish 토픽:
  - `/dsr01/pharmacy/events`
  - `/dsr01/pharmacy/medicine`
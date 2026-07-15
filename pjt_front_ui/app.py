"""
약국 업무 보조 로봇 - 웹 대시보드 + ROS2 브릿지
=================================================

이번 버전에서 바뀐 핵심
-----------------------
실제 DB API를 호출해보고 확인된 사실 두 가지를 반영했다.

1. 필드명이 우리가 가정했던 것과 다르다 (매핑만 하면 되는 부분)
     GET /api/medicine/  ->  id, medicine_name, stock, min_stock, storage_location, dispensing_location, lid_type
     GET /api/events/    ->  id, prescription_name, status, created_at, items:[{id, medicine_name, quantity, order}]

2. status가 "약 하나마다"가 아니라 "처방전(이벤트) 전체에 하나"뿐이다.
     -> 검수완료 버튼도 이제 약 하나하나가 아니라 "처방전 카드 하나당 하나"로 바뀐다.
     -> 이전 버전의 "task_id 단위 검수완료"는 전부 "event_id(=prescription_id) 단위"로 변경.

확정된 것 / 아직 추정인 것
--------------------------
- GET /api/medicine/         : 실제 응답 확인 완료 (그대로 매핑)
- GET /api/events/           : 실제 응답 확인 완료 (그대로 매핑)
- POST /api/tasks/status/    : 요청 형태 확정됨 -> {"event_id": <int>, "status": "PROCESSING" | "DONE"}
- POST /api/prescriptions/   : ⚠ 아직 실제 요청/응답 예시를 못 받음. 지금까지 관찰된 네이밍
                                컨벤션(prescription_name, medicine_name, quantity)에 맞춰 추정해서
                                구현했다. 실제 예시 받으면 create_prescription_remote()의 body만
                                맞추면 된다.
- status 값 종류             : 지금까지 확인된 건 PROCESSING, DONE 뿐. "대기(제조대기)"에 해당하는
                                값이 있는지는 미확인 -> 일단 모르는 값은 '제조중'으로 취급(map_status).

아키텍처 변경: ROS2 발행은 이제 DB API(Django) 쪽에서 직접 처리
-------------------------------------------------------------
기존에는 이 Flask 앱(ui_alarm_node)이 DB API 응답을 받은 뒤 ROS2로 발행까지
했었는데, DB 담당자 쪽에서 직접 rclpy 노드를 하나 띄워서 처리하는 것으로
결정했다. 그래서 이 파일은 이제 "구독(로봇 상태 -> 화면 반영)"만 담당한다.

    브라우저 --(fetch)--> 이 Flask 앱 --(requests.get/post)--> DB API(Django)
                                                                    │
                                                                    └--(rclpy publish)--> ROS2 topic

DB 담당자에게 전달할 것: /pharmacy/new_prescription, /pharmacy/inspection_done
발행 로직 템플릿은 django_ros_bridge_example.py 참고 (별도 파일로 전달).

핵심 개념: 브라우저는 ROS2에 직접 접근할 수 없다
--------------------------------------------------
브라우저 JS는 HTTP/WebSocket만 가능하고 ROS2가 쓰는 DDS 프로토콜에는
접근할 수 없다. 그래서 누군가는(이 Flask 앱이든, DB API든) rclpy 노드를
직접 띄워서 "번역"해줘야 한다 - 이번 결정은 그 역할을 DB API 쪽으로 옮긴 것뿐,
"브라우저가 직접 못 한다"는 원칙 자체는 그대로다.

GET /api/tasks/next/ 관련 참고
------------------------------
이 엔드포인트가 GET 전용(POST 없음)이고 "대기 중인 작업이 없습니다" 형태로 응답하는 걸로 봐서,
로봇(task_manager_node)이 이 API를 직접 폴링해서 다음 작업을 가져가는 방식으로 보인다.

페이지 구성
-----------
- /dashboard    : 제조중 / 제조완료 (환자 이름 표시)
- /inventory    : 약 종류별 재고 현황
- /production   : 처방전별 상태 + 검수완료 버튼 (처방전 단위)
- /prescription : 처방전 입력 (환자 이름 + 약 이름/개수, 재고 부족 시 알럿)

ROS2 토픽
---------
[구독] (이 앱이 직접 구독 - 로봇 상태를 화면에 실시간 반영)
  /pharmacy/task_start         String(JSON) - 로봇이 특정 처방전 조제 시작
  /pharmacy/task_done          String(JSON) - (로봇 측) 최종 완료 (안전망용)
  /pharmacy/remaining_count    String(JSON) - 재고 수량 갱신

[발행] (★ 이제 이 앱이 아니라 DB API(Django) 쪽에서 처리 ★)
  /pharmacy/new_prescription   String(JSON) - 처방전 등록 처리 성공 시
  /pharmacy/inspection_done    String(JSON) - 검수완료(status=DONE) 처리 성공 시
"""

import json
import os
import threading
from datetime import datetime, timezone, timedelta

import requests

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from flask import Flask, render_template, request, jsonify, redirect, url_for
from flask_socketio import SocketIO

# ---------------------------------------------------------------
# Flask / SocketIO
# ---------------------------------------------------------------
app = Flask(__name__)
app.config["SECRET_KEY"] = os.getenv(
    "PHARMACY_UI_SECRET_KEY",
    "development-only-secret-key",
)
socketio = SocketIO(app, async_mode="threading", cors_allowed_origins="*")

# ---------------------------------------------------------------
# DB API 주소
# ---------------------------------------------------------------
DB_API_BASE_URL = os.getenv(
    "PHARMACY_BACKEND_URL",
    "http://127.0.0.1:8000",
).rstrip("/")
MEDICINE_ENDPOINT = f"{DB_API_BASE_URL}/api/medicine/"
DEFAULT_LOW_STOCK_THRESHOLD = 5

EVENTS_ENDPOINT = f"{DB_API_BASE_URL}/api/events/"                 # 조회는 여기 (GET)
PRESCRIPTIONS_CREATE_ENDPOINT = f"{DB_API_BASE_URL}/api/prescriptions/"  # 등록은 여기 (POST 전용)
TASK_STATUS_ENDPOINT = f"{DB_API_BASE_URL}/api/tasks/status/"


# ---------------------------------------------------------------
# status 값 매핑 (DB API의 대문자 값 -> 우리 화면의 3단계 라벨)
# ---------------------------------------------------------------
# ---------------------------------------------------------------
# DB가 UTC(끝에 'Z')로 주는 시각을 한국 시간(KST, UTC+9)으로 변환
# ---------------------------------------------------------------
KST = timezone(timedelta(hours=9))


def format_kst(iso_str):
    """'2026-07-07T07:17:21.936771Z' (UTC) -> '16:17' (KST)"""
    if not iso_str:
        return ""
    try:
        dt = datetime.fromisoformat(iso_str.replace("Z", "+00:00"))
        return dt.astimezone(KST).strftime("%H:%M")
    except ValueError:
        print(f"[WARN] created_at 파싱 실패: {iso_str}")
        return iso_str


def map_status(remote_status):
    """
    '제조대기' 상태는 프론트에서 안 쓰기로 함 -> 처방전 등록되는 순간부터
    전부 '제조중'으로 취급하고 검수완료 버튼도 바로 노출한다.
    (WAITING/PROCESSING 둘 다 'production'으로 합침)
    """
    mapping = {
        "WAITING": "production",
        "PROCESSING": "production",
        "DONE": "done",
    }
    if remote_status not in mapping:
        print(f"[WARN] 알 수 없는 status 값: {remote_status} -> 'production'으로 임시 처리")
    return mapping.get(remote_status, "production")


# ---------------------------------------------------------------
# DB API 호출 헬퍼
# ---------------------------------------------------------------
def get_inventory():
    """
    GET /api/medicine/ -> 화면에서 쓰는 형태로 매핑.

    적재소(storage_stock) vs 제조기(dispensing_stock) 구분됨.
    UI(재고현황, 처방전 등록 시 재고체크)는 '적재소' 기준으로 판단하므로
    storage_stock을 count로 매핑한다. dispensing_stock(제조기 내부 재고)은
    로봇이 자동으로 리필/관리하는 내부 값이라 화면에는 노출하지 않는다.
    (DB 쪽 확인 완료 - storage_stock이 정확한 필드명)
    """
    try:
        res = requests.get(MEDICINE_ENDPOINT, timeout=5)
        res.raise_for_status()
        raw = res.json()
    except requests.exceptions.RequestException as e:
        print(f"[ERROR] 재고 조회 실패: {e}")
        return []

    # 응답이 리스트가 아니라 {"results": [...]} 형태(페이지네이션)로 올 수도 있어서 방어
    if isinstance(raw, dict) and "results" in raw:
        raw = raw["results"]

    result = []
    for m in raw:
        if "medicine_number" not in m or "medicine_name" not in m:
            print(f"[WARN] 필수 필드 누락, 이 항목 스킵: {m}")
            continue
        result.append({
            "med_id": m["medicine_number"],
            "name": m["medicine_name"],
            "count": m.get("storage_stock", 0),
            "threshold": DEFAULT_LOW_STOCK_THRESHOLD,
        })
    return result


def get_prescriptions():
    """
    GET /api/events/ -> 처방전(=이벤트) 목록.
    status는 이벤트 전체에 하나뿐이라, items는 '표시용 목록'일 뿐 개별 상태/버튼이 없다.
    """
    try:
        res = requests.get(EVENTS_ENDPOINT, timeout=5)
        res.raise_for_status()
        raw = res.json()
    except requests.exceptions.RequestException as e:
        print(f"[ERROR] 처방전 조회 실패: {e}")
        return []

    out = []
    for ev in raw:
        created_at = format_kst(ev.get("created_at", ""))
        out.append({
            "id": ev["id"],
            "patient_name": ev.get("prescription_name") or "-",
            "created_at": created_at,
            "status": map_status(ev.get("status")),
            "items": [
                {"med_name": it["medicine_name"], "qty": it["quantity"]}
                for it in ev.get("items", [])
            ],
        })
    return out


def create_prescription_remote(patient_name, items):
    """
    POST /api/prescriptions/ -> 처방전 등록.
    ⚠ 실제 요청/응답 예시 미확인 - 관찰된 네이밍 컨벤션에 맞춰 추정 구현.
       실제 예시 받으면 아래 body만 맞추면 된다.
    """
    body = {
        "prescription_name": patient_name,
        "items": [
            {"medicine_name": item["med_name"], "quantity": item["qty"]}
            for item in items
        ],
    }
    try:
        res = requests.post(PRESCRIPTIONS_CREATE_ENDPOINT, json=body, timeout=5)
        return res.status_code, res.json()
    except requests.exceptions.RequestException as e:
        print(f"[ERROR] 처방전 등록 실패: {e}")
        return 503, {"message": "DB 서버에 연결할 수 없습니다."}


def update_event_status_remote(event_id, status):
    """POST /api/tasks/status/ -> {"event_id": ..., "status": "PROCESSING" | "DONE"} (형태 확정됨)"""
    try:
        res = requests.post(
            TASK_STATUS_ENDPOINT,
            json={"event_id": event_id, "status": status},
            timeout=5,
        )
        return res.status_code, res.json()
    except requests.exceptions.RequestException as e:
        print(f"[ERROR] 상태 변경 실패: {e}")
        return 503, {"message": "DB 서버에 연결할 수 없습니다."}


def dashboard_events_as_list():
    """대시보드용: 처방전(이벤트) 단위로 평탄화."""
    out = []
    for ev in get_prescriptions():
        med_summary = ", ".join(it["med_name"] for it in ev["items"]) or "-"
        out.append({
            "id": ev["id"],
            "patient_name": ev["patient_name"],
            "med_name": med_summary,
            "status": ev["status"],
        })
    return out


# ---------------------------------------------------------------
# ROS2 노드 : 구독(실시간 브로드캐스트)만 담당.
# 발행(new_prescription, inspection_done)은 DB 담당자 쪽(Django)에서
# 직접 ROS2 노드를 띄워서 처리하기로 결정 (아래 별도 안내 참고)
# ---------------------------------------------------------------
class PharmacyBridgeNode(Node):
    def __init__(self):
        super().__init__("ui_alarm_node")

        self.create_subscription(String, "/pharmacy/task_start", self.on_task_start, 10)
        self.create_subscription(String, "/pharmacy/task_done", self.on_task_done, 10)
        self.create_subscription(String, "/pharmacy/remaining_count", self.on_remaining_count, 10)

        self.get_logger().info("ui_alarm_node (pharmacy bridge) ready.")

    def on_task_start(self, msg: String):
        payload = self._parse(msg)
        if payload is not None:
            socketio.emit("task_start", payload)

    def on_task_done(self, msg: String):
        payload = self._parse(msg)
        if payload is not None:
            socketio.emit("task_done", payload)

    def on_remaining_count(self, msg: String):
        payload = self._parse(msg)
        if payload is None:
            return
        threshold = payload.get("threshold", 5)
        if payload.get("count", 0) <= threshold:
            socketio.emit("stock_alert", payload)
        socketio.emit("remaining_count", payload)

    def _parse(self, msg: String):
        try:
            return json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f"invalid JSON: {msg.data}")
            return None


ros_node = None  # __main__ 에서 PharmacyBridgeNode 인스턴스로 채워짐


# ---------------------------------------------------------------
# 페이지 라우트
# ---------------------------------------------------------------
@app.route("/")
def root():
    return redirect(url_for("dashboard"))


@app.route("/dashboard")
def dashboard():
    events = dashboard_events_as_list()
    return render_template("dashboard.html", page_title="대시보드", active_page="dashboard", tasks=events)


@app.route("/inventory")
def inventory_page():
    inv = get_inventory()
    return render_template("inventory.html", page_title="재고 현황", active_page="inventory", inventory=inv)


@app.route("/production")
def production():
    rx = get_prescriptions()
    return render_template("production.html", page_title="제조 현황", active_page="production", prescriptions=rx)


@app.route("/prescription")
def prescription():
    inv = get_inventory()
    return render_template("prescription.html", page_title="처방전 입력", active_page="prescription", inventory=inv)


# ---------------------------------------------------------------
# API
# ---------------------------------------------------------------
@app.route("/api/prescriptions", methods=["POST"])
def create_prescription():
    body = request.get_json(force=True)
    patient_name = (body.get("patient_name") or "").strip()
    items = body.get("items") or []

    if not patient_name or not items:
        return jsonify({"message": "환자 이름과 약 정보를 입력해주세요."}), 400

    # 프론트는 med_id(숫자)로 보내지만, DB API 추정 스펙은 medicine_name 기반이라 여기서 변환
    # (str로 통일 비교 -> 프론트에서 문자열/숫자 어느 쪽으로 와도 안전하게 매칭)
    inventory_by_id = {str(m["med_id"]): m["name"] for m in get_inventory()}
    resolved_items = []
    for item in items:
        med_name = inventory_by_id.get(str(item["med_id"]))
        if med_name is None:
            return jsonify({"message": f"존재하지 않는 약품입니다: {item['med_id']}"}), 400
        resolved_items.append({"med_name": med_name, "qty": item["qty"]})

    status_code, result = create_prescription_remote(patient_name, resolved_items)

    if status_code == 201:
        # ROS2 발행은 이제 DB API(Django) 쪽에서 직접 처리 (아래 안내 참고)
        socketio.emit("prescription_created", result)

    return jsonify(result), status_code


@app.route("/api/tasks/<int:event_id>/inspect", methods=["POST"])
def inspect_task(event_id):
    """검수완료 - 이제 처방전(이벤트) 단위로 하나만 호출하면 된다."""
    status_code, result = update_event_status_remote(event_id, "DONE")

    if status_code == 200:
        payload = {"event_id": event_id, **(result if isinstance(result, dict) else {})}
        # ROS2 발행은 이제 DB API(Django) 쪽에서 직접 처리 (아래 안내 참고)
        socketio.emit("inspection_done", payload)
        socketio.emit("task_done", payload)

    return jsonify(result), status_code


@socketio.on("connect")
def handle_connect():
    print("client connected")


# ---------------------------------------------------------------
# 엔트리포인트
# ---------------------------------------------------------------
if __name__ == "__main__":
    rclpy.init()
    ros_node = PharmacyBridgeNode()

    spin_thread = threading.Thread(target=lambda: rclpy.spin(ros_node), daemon=True)
    spin_thread.start()

    try:
        debug_enabled = os.getenv("FLASK_DEBUG", "0") == "1"
        socketio.run(
            app,
            host="0.0.0.0",
            port=5000,
            debug=debug_enabled,
            use_reloader=False,
        )
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()

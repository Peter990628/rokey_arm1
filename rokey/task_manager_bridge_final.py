import json
from urllib import error, request

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


DEFAULT_BACKEND_BASE_URL = "http://127.0.0.1:8000/api"
DEFAULT_EVENTS_TOPIC = "/dsr01/pharmacy/events"
DEFAULT_MEDICINE_TOPIC = "/dsr01/pharmacy/medicine"
DEFAULT_REFILL_REQUIRED_MEDICINE_TOPIC = (
    "/dsr01/pharmacy/refill_required_medicine"
)

ACTIVE_REFILL_ITEM_STATUS = "REFILL_REQUIRED"

# 로봇 PourPills.set_task_from_data()가 요구하는 Medicine 필드
ROBOT_TASK_FIELDS = (
    "medicine_number",
    "medicine_name",
    "storage_x", "storage_y", "storage_z",
    "storage_rx", "storage_ry", "storage_rz",
    "dispensing_x", "dispensing_y", "dispensing_z",
    "dispensing_rx", "dispensing_ry", "dispensing_rz",
    "bottle_tip_offset_x",
    "bottle_tip_offset_y",
    "bottle_tip_offset_z",
    "drawer_x", "drawer_y", "drawer_z",
    "drawer_rx", "drawer_ry", "drawer_rz",
    "lid_type",
    "storage_stock",
    "dispensing_stock",
)


def normalize_name(value):
    if not isinstance(value, str):
        return None
    normalized = value.strip()
    return normalized or None


class TaskManagerBridge(Node):
    """
    UI가 POST /api/prescriptions/로 처방전을 등록하면 Django가 Event/EventItem을 만든다.

    이 노드는 주기적으로:
      1. GET /api/events/
      2. GET /api/medicine/
      3. Event.status == REFILL_REQUIRED인 이벤트만 선택
      4. EventItem.status == REFILL_REQUIRED인 항목만 선택
      5. medicine_name만 사용해 Medicine 매칭
      6. 로봇 제어용 평평한 JSON 배열 publish
    """

    def __init__(self):
        super().__init__("task_manager_bridge")

        self.declare_parameter("backend_base_url", DEFAULT_BACKEND_BASE_URL)
        self.declare_parameter("poll_interval_sec", 1.0)
        self.declare_parameter("request_timeout_sec", 2.0)
        self.declare_parameter("events_topic", DEFAULT_EVENTS_TOPIC)
        self.declare_parameter("medicine_topic", DEFAULT_MEDICINE_TOPIC)
        self.declare_parameter(
            "refill_required_medicine_topic",
            DEFAULT_REFILL_REQUIRED_MEDICINE_TOPIC,
        )

        self.backend_base_url = (
            self.get_parameter("backend_base_url")
            .get_parameter_value()
            .string_value
            .rstrip("/")
        )
        self.poll_interval_sec = (
            self.get_parameter("poll_interval_sec")
            .get_parameter_value()
            .double_value
        )
        self.request_timeout_sec = (
            self.get_parameter("request_timeout_sec")
            .get_parameter_value()
            .double_value
        )
        self.events_topic = (
            self.get_parameter("events_topic")
            .get_parameter_value()
            .string_value
        )
        self.medicine_topic = (
            self.get_parameter("medicine_topic")
            .get_parameter_value()
            .string_value
        )
        self.refill_required_medicine_topic = (
            self.get_parameter("refill_required_medicine_topic")
            .get_parameter_value()
            .string_value
        )

        self.events_pub = self.create_publisher(String, self.events_topic, 10)
        self.medicine_pub = self.create_publisher(String, self.medicine_topic, 10)
        self.refill_pub = self.create_publisher(
            String,
            self.refill_required_medicine_topic,
            10,
        )

        self._logged_errors = set()
        self._last_task_signature = None

        self.create_timer(self.poll_interval_sec, self._poll_backend)

        self.get_logger().info(
            "task_manager_bridge 시작: "
            f"backend={self.backend_base_url}, "
            f"poll={self.poll_interval_sec:.1f}s, "
            f"output={self.refill_required_medicine_topic}"
        )

    def _poll_backend(self):
        events_data = self._get_and_publish(
            label="events",
            path="/events/",
            publisher=self.events_pub,
        )
        medicines_data = self._get_and_publish(
            label="medicine",
            path="/medicine/",
            publisher=self.medicine_pub,
        )

        if events_data is None or medicines_data is None:
            return

        tasks = self._build_refill_tasks(events_data, medicines_data)
        if not tasks:
            self._last_task_signature = None
            return

        msg = String()
        msg.data = json.dumps(tasks, ensure_ascii=False)
        self.refill_pub.publish(msg)

        # publish는 로봇이 늦게 실행될 경우를 위해 polling마다 유지한다.
        # 로그만 작업 구성이 바뀔 때 출력한다.
        signature = json.dumps(tasks, ensure_ascii=False, sort_keys=True)
        if signature != self._last_task_signature:
            summary = [
                f"{task['medicine_name']}({task['lid_type']})"
                for task in tasks
            ]
            self.get_logger().info(
                f"리필 작업 publish: {len(tasks)}개, {summary}"
            )
            self._last_task_signature = signature

    def _build_refill_tasks(self, events_data, medicines_data):
        events = self._extract_records(events_data)
        medicines = self._extract_records(medicines_data)

        medicine_by_name = {}

        for medicine in medicines:
            if not isinstance(medicine, dict):
                continue

            medicine_name = normalize_name(medicine.get("medicine_name"))
            if medicine_name:
                medicine_by_name[medicine_name] = medicine

        # 동일 약이 여러 처방전에 부족해도 리필 로봇 작업은 약 종류당 한 번 생성한다.
        tasks_by_medicine = {}

        for event in events:
            if not isinstance(event, dict):
                continue

            event_id = event.get("id")
            items = event.get("items", [])
            if not isinstance(items, list):
                self._log_once(
                    ("invalid_items", event_id),
                    f"Event.items가 list가 아님: event_id={event_id}",
                )
                continue

            for item in items:
                if not isinstance(item, dict):
                    continue

                item_status = str(item.get("status", "")).strip().upper()
                if item_status != ACTIVE_REFILL_ITEM_STATUS:
                    continue

                medicine = self._find_medicine_by_name(
                    item,
                    medicine_by_name,
                )
                if medicine is None:
                    self._log_once(
                        (
                            "medicine_not_found",
                            event_id,
                            item.get("id"),
                            item.get("medicine_number"),
                            item.get("medicine_name"),
                        ),
                        "EventItem에 대응되는 Medicine을 찾지 못함: "
                        f"event_id={event_id}, item={item}",
                    )
                    continue

                task = self._make_robot_task(medicine)
                if task is None:
                    continue

                task_key = normalize_name(task["medicine_name"])
                existing = tasks_by_medicine.get(task_key)

                if existing is None:
                    # 로봇은 아래 추가 메타데이터를 무시해도 된다.
                    task["source_event_ids"] = []
                    task["source_event_item_ids"] = []
                    task["requested_quantity_total"] = 0
                    tasks_by_medicine[task_key] = task
                    existing = task

                if event_id is not None and event_id not in existing["source_event_ids"]:
                    existing["source_event_ids"].append(event_id)

                item_id = item.get("id")
                if (
                    item_id is not None
                    and item_id not in existing["source_event_item_ids"]
                ):
                    existing["source_event_item_ids"].append(item_id)

                try:
                    existing["requested_quantity_total"] += int(
                        item.get("quantity", 0)
                    )
                except (TypeError, ValueError):
                    pass

        return list(tasks_by_medicine.values())

    def _find_medicine_by_name(
        self,
        item,
        medicine_by_name,
    ):
        # EventItem의 medicine_name과 Medicine의 medicine_name만 비교한다.
        medicine_name = normalize_name(item.get("medicine_name"))
        if medicine_name:
            return medicine_by_name.get(medicine_name)

        return None

    def _make_robot_task(self, medicine):
        task = {field: medicine.get(field) for field in ROBOT_TASK_FIELDS}

        if task["medicine_number"] is None:
            task["medicine_number"] = medicine.get("id")

        if isinstance(task.get("lid_type"), str):
            task["lid_type"] = task["lid_type"].strip().lower()

        missing = [field for field in ROBOT_TASK_FIELDS if task.get(field) is None]
        if missing:
            self._log_once(
                (
                    "missing_fields",
                    medicine.get("medicine_name"),
                    tuple(missing),
                ),
                "로봇 작업에 필요한 Medicine 필드 누락: "
                f"medicine={medicine.get('medicine_name')}, missing={missing}",
            )
            return None

        try:
            task["medicine_number"] = int(task["medicine_number"])
        except (TypeError, ValueError):
            self._log_once(
                ("invalid_medicine_number", task.get("medicine_name")),
                f"medicine_number가 정수가 아님: {task.get('medicine_number')}",
            )
            return None

        if task["lid_type"] not in ("pull", "spin"):
            self._log_once(
                (
                    "unsupported_lid_type",
                    task["medicine_name"],
                    task["lid_type"],
                ),
                "지원하지 않는 lid_type: "
                f"medicine={task['medicine_name']}, "
                f"lid_type={task['lid_type']}",
            )
            return None

        return task

    def _get_and_publish(self, label, path, publisher):
        status_code, data = self._get_json(path)

        if status_code != 200:
            self._log_once(
                (
                    "get_failed",
                    label,
                    status_code,
                    json.dumps(data, ensure_ascii=False, sort_keys=True),
                ),
                f"{label} GET 실패: status={status_code}, body={data}",
            )
            return None

        msg = String()
        msg.data = json.dumps(data, ensure_ascii=False)
        publisher.publish(msg)
        return data

    def _get_json(self, path):
        url = f"{self.backend_base_url}{path}"
        req = request.Request(
            url,
            headers={"Accept": "application/json"},
            method="GET",
        )

        try:
            with request.urlopen(
                req,
                timeout=self.request_timeout_sec,
            ) as response:
                raw = response.read().decode("utf-8")
                return response.status, self._decode_json(raw)
        except error.HTTPError as exc:
            raw = exc.read().decode("utf-8")
            return exc.code, self._decode_json(raw)
        except (error.URLError, TimeoutError) as exc:
            return 0, {"error": str(exc)}

    @staticmethod
    def _extract_records(data):
        if isinstance(data, list):
            return data
        if isinstance(data, dict) and isinstance(data.get("results"), list):
            return data["results"]
        return []

    @staticmethod
    def _decode_json(raw):
        if not raw:
            return {}
        try:
            return json.loads(raw)
        except json.JSONDecodeError:
            return {"raw": raw}

    def _log_once(self, key, message):
        if key in self._logged_errors:
            return
        self._logged_errors.add(key)
        self.get_logger().warning(message)


def main(args=None):
    rclpy.init(args=args)
    node = TaskManagerBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
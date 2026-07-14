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

REFILL_TASK_FIELDS = (
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

# UI와 DB 없이 Task Manager의 매칭 로직을 검증하기 위한 가상 Event 응답.
TEST_EVENTS = [
    {
        "id": 9001,
        "prescription_name": "통합 테스트 처방전",
        "status": "WAITING",
        "created_at": "2026-07-14T00:00:00+09:00",
        "items": [
            {
                "id": 9101,
                "medicine_name": "별사탕",
                "quantity": 1,
                "order": 1,
                "status": "REFILL_REQUIRED",
            }
        ],
    }
]

# UI와 DB 없이 /api/medicine/ 응답을 대신하는 테스트 Medicine 데이터.
TEST_MEDICINES = [
    {
        "medicine_number": 3,
        "medicine_name": "별사탕",
        "storage_x": 440.45,
        "storage_y": 401.24,
        "storage_z": 219.77,
        "storage_rx": 47.73,
        "storage_ry": -179.79,
        "storage_rz": 47.19,
        "dispensing_x": -33.32,
        "dispensing_y": 29.23,
        "dispensing_z": 41.55,
        "dispensing_rx": 34.95,
        "dispensing_ry": 62.8,
        "dispensing_rz": -132.18,
        "bottle_tip_offset_x": 0.0,
        "bottle_tip_offset_y": 25.0,
        "bottle_tip_offset_z": -42.0,
        "drawer_x": -39.25,
        "drawer_y": 21.17,
        "drawer_z": 95.86,
        "drawer_rx": -65.24,
        "drawer_ry": -49.36,
        "drawer_rz": -32.78,
        "lid_type": "pull",
        "storage_stock": 123,
        "dispensing_stock": 123,
    }
]


class TaskManagerBridge(Node):
    def __init__(self):
        super().__init__("task_manager_bridge")

        self.declare_parameter("test_mode", True)
        self.declare_parameter("test_publish_delay_sec", 3.0)
        self.declare_parameter("backend_base_url", DEFAULT_BACKEND_BASE_URL)
        self.declare_parameter("poll_interval_sec", 1.0)
        self.declare_parameter("request_timeout_sec", 2.0)
        self.declare_parameter("events_topic", DEFAULT_EVENTS_TOPIC)
        self.declare_parameter("medicine_topic", DEFAULT_MEDICINE_TOPIC)
        self.declare_parameter(
            "refill_required_medicine_topic",
            DEFAULT_REFILL_REQUIRED_MEDICINE_TOPIC,
        )

        self.test_mode = (
            self.get_parameter("test_mode").get_parameter_value().bool_value
        )
        self.test_publish_delay_sec = (
            self.get_parameter("test_publish_delay_sec")
            .get_parameter_value()
            .double_value
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
            self.get_parameter("events_topic").get_parameter_value().string_value
        )
        self.medicine_topic = (
            self.get_parameter("medicine_topic").get_parameter_value().string_value
        )
        self.refill_required_medicine_topic = (
            self.get_parameter("refill_required_medicine_topic")
            .get_parameter_value()
            .string_value
        )

        self.events_pub = self.create_publisher(String, self.events_topic, 10)
        self.medicine_pub = self.create_publisher(String, self.medicine_topic, 10)
        self.refill_required_medicine_pub = self.create_publisher(
            String,
            self.refill_required_medicine_topic,
            10,
        )

        self._logged_errors = set()
        self._test_published = False
        self._test_timer = None

        if self.test_mode:
            # 로봇 Subscriber가 먼저 생성될 시간을 주고 한 번만 publish한다.
            self._test_timer = self.create_timer(
                self.test_publish_delay_sec,
                self._publish_test_data_once,
            )
            self.get_logger().info(
                "Task Manager 2-node 테스트 모드: "
                f"{self.test_publish_delay_sec:.1f}초 후 테스트 작업 1회 발행"
            )
        else:
            self.create_timer(self.poll_interval_sec, self._poll_backend)
            self.get_logger().info(
                f"Task Manager 실제 DB 모드: backend={self.backend_base_url}"
            )

    def _publish_test_data_once(self):
        if self._test_published:
            return

        self._test_published = True

        # 실제 모드와 같은 데이터 흐름을 사용한다.
        self._publish_json(self.events_pub, TEST_EVENTS)
        self._publish_json(self.medicine_pub, TEST_MEDICINES)
        self._publish_refill_required_medicines(
            TEST_EVENTS,
            TEST_MEDICINES,
        )

        self.get_logger().info(
            "테스트 Event와 Medicine 매칭 및 로봇 작업 토픽 발행 완료"
        )

        if self._test_timer is not None:
            self.destroy_timer(self._test_timer)
            self._test_timer = None

    def _poll_backend(self):
        events_data = self._get_and_publish(
            "/events/",
            self.events_pub,
        )
        medicine_data = self._get_and_publish(
            "/medicine/",
            self.medicine_pub,
        )

        if events_data is None or medicine_data is None:
            return

        self._publish_refill_required_medicines(
            events_data,
            medicine_data,
        )

    def _get_and_publish(self, path: str, publisher):
        status_code, data = self._get_json(path)
        if status_code != 200:
            error_key = (path, status_code, json.dumps(data, ensure_ascii=False))
            if error_key not in self._logged_errors:
                self.get_logger().warning(
                    f"GET 실패: path={path}, status={status_code}, body={data}"
                )
                self._logged_errors.add(error_key)
            return None

        self._publish_json(publisher, data)
        return data

    @staticmethod
    def _extract_records(data):
        if isinstance(data, list):
            return data
        if isinstance(data, dict) and isinstance(data.get("results"), list):
            return data["results"]
        return []

    @staticmethod
    def _publish_json(publisher, data):
        msg = String()
        msg.data = json.dumps(data, ensure_ascii=False)
        publisher.publish(msg)

    def _publish_refill_required_medicines(self, events_data, medicine_data):
        events = self._extract_records(events_data)
        medicines = self._extract_records(medicine_data)

        # medicine_name -> Medicine 전체 정보 색인.
        medicine_by_name = {
            medicine.get("medicine_name"): medicine
            for medicine in medicines
            if isinstance(medicine, dict) and medicine.get("medicine_name")
        }

        refill_tasks = []
        added_names = set()

        for event in events:
            if not isinstance(event, dict):
                continue

            items = event.get("items", [])
            if not isinstance(items, list):
                continue

            for item in items:
                if not isinstance(item, dict):
                    continue

                item_status = str(item.get("status", "")).strip().upper()
                if item_status == "READY":
                    continue

                medicine_name = item.get("medicine_name")
                if not medicine_name or medicine_name in added_names:
                    continue

                medicine = medicine_by_name.get(medicine_name)
                if medicine is None:
                    self.get_logger().warning(
                        f"Event 약품을 Medicine에서 찾지 못함: {medicine_name}"
                    )
                    continue

                task = self._build_refill_task(medicine)
                if task is None:
                    continue

                refill_tasks.append(task)
                added_names.add(medicine_name)

                self.get_logger().info(
                    f"매칭 성공: Event medicine_name={medicine_name} "
                    f"-> Medicine medicine_number={task['medicine_number']}"
                )

        if not refill_tasks:
            self.get_logger().warning("발행할 리필 작업이 없음")
            return

        self._publish_json(
            self.refill_required_medicine_pub,
            refill_tasks,
        )

        self.get_logger().info(
            f"{self.refill_required_medicine_topic} 발행: "
            f"{len(refill_tasks)}개"
        )

    def _build_refill_task(self, medicine: dict):
        task = {field: medicine.get(field) for field in REFILL_TASK_FIELDS}

        if task["medicine_number"] is None:
            task["medicine_number"] = medicine.get("id")

        if isinstance(task.get("lid_type"), str):
            task["lid_type"] = task["lid_type"].strip().lower()

        missing_fields = [
            field
            for field in REFILL_TASK_FIELDS
            if task.get(field) is None
        ]
        if missing_fields:
            self.get_logger().warning(
                f"로봇 작업 생성 실패: medicine={medicine.get('medicine_name')}, "
                f"누락 필드={missing_fields}"
            )
            return None

        if task["lid_type"] not in ("pull", "spin"):
            self.get_logger().warning(
                f"지원하지 않는 lid_type: {task['lid_type']}"
            )
            return None

        return task

    def _get_json(self, path: str):
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
    def _decode_json(raw: str):
        if not raw:
            return {}
        try:
            return json.loads(raw)
        except json.JSONDecodeError:
            return {"raw": raw}


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
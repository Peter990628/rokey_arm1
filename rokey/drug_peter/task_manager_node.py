# task_manager_node.py
#
# Django pharmacy_backend <-> ROS2 bridge node.
# - Fetches the next WAITING prescription task from the backend.
# - Publishes the task as JSON for robot-control nodes.
# - Publishes refill-needed signals for compatibility with simple Bool nodes.

import json
from urllib import error, request

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import String


DEFAULT_BACKEND_BASE_URL = "http://127.0.0.1:8000/api"
DEFAULT_ROBOT_NS = "/dsr01"


MEDICINE_FLAG_TOPICS = {
    "타이레놀": "dispensary_a_num",
    "탁센": "dispensary_b_num",
    "브루펜": "dispensary_c_num",
    "활명수": "dispensary_d_num",
}


class Task_Manager(Node):
    def __init__(self):
        super().__init__("task_manager")

        self.declare_parameter("backend_base_url", DEFAULT_BACKEND_BASE_URL)
        self.declare_parameter("robot_ns", DEFAULT_ROBOT_NS)
        self.declare_parameter("poll_interval_sec", 1.0)
        self.declare_parameter("request_timeout_sec", 2.0)

        self.backend_base_url = (
            self.get_parameter("backend_base_url")
            .get_parameter_value()
            .string_value
            .rstrip("/")
        )
        self.robot_ns = self._normalize_ns(
            self.get_parameter("robot_ns").get_parameter_value().string_value
        )
        self.poll_interval_sec = (
            self.get_parameter("poll_interval_sec").get_parameter_value().double_value
        )
        self.request_timeout_sec = (
            self.get_parameter("request_timeout_sec").get_parameter_value().double_value
        )

        self.last_published_event_id = None
        self._logged_no_task = False

        self.prescription_pub = self.create_publisher(
            String,
            self._topic("prescription"),
            10,
        )
        self.refill_request_pub = self.create_publisher(
            String,
            self._topic("refill_request"),
            10,
        )
        self.refill_needed_pub = self.create_publisher(
            Bool,
            self._topic("refill_needed"),
            10,
        )
        self.task_state_pub = self.create_publisher(
            String,
            self._topic("task_state"),
            10,
        )

        self.refill_flag_pubs = {
            medicine_name: self.create_publisher(
                Bool,
                self._topic(topic_name),
                10,
            )
            for medicine_name, topic_name in MEDICINE_FLAG_TOPICS.items()
        }

        self.create_timer(self.poll_interval_sec, self._poll_next_task)

        self.get_logger().info(
            f"task_manager_node started. backend={self.backend_base_url}, "
            f"robot_ns={self.robot_ns}"
        )

    def _normalize_ns(self, namespace: str) -> str:
        namespace = namespace.strip()
        if not namespace:
            return ""
        if not namespace.startswith("/"):
            namespace = "/" + namespace
        return namespace.rstrip("/")

    def _topic(self, name: str) -> str:
        if self.robot_ns:
            return f"{self.robot_ns}/{name}"
        return name

    def _poll_next_task(self):
        status_code, task = self._get_json("/tasks/next/")

        if status_code == 404:
            if not self._logged_no_task:
                self.get_logger().info("대기 중인 처방 작업이 없습니다.")
                self._logged_no_task = True
            return

        if status_code != 200:
            self.get_logger().warn(f"다음 작업 조회 실패: {status_code} {task}")
            return

        self._logged_no_task = False

        if not self._is_valid_task(task):
            self.get_logger().error(f"백엔드 작업 데이터 형식이 올바르지 않습니다: {task}")
            return

        event_id = task["event_id"]

        if event_id == self.last_published_event_id:
            return

        self.last_published_event_id = event_id

        self._publish_task(task)
        self._publish_task_state(event_id, task.get("status"), "작업 publish 완료")

    def _is_valid_task(self, task) -> bool:
        if not isinstance(task, dict):
            return False
        if "event_id" not in task or "items" not in task:
            return False
        if not isinstance(task["items"], list):
            return False
        return True

    def _publish_task(self, task: dict):
        msg = String()
        msg.data = json.dumps(task, ensure_ascii=False)
        self.prescription_pub.publish(msg)

        self.get_logger().info(
            f"처방 작업 publish: event_id={task['event_id']}, "
            f"items={len(task['items'])}"
        )

        self._publish_refill_flags(task["items"])
        self._publish_refill_requests(task)
        self._publish_refill_needed(task["items"])

    def _publish_refill_flags(self, items: list):
        for pub in self.refill_flag_pubs.values():
            self._publish_bool(pub, False)

        for item in items:
            medicine_name = item.get("medicine_name")
            refill_needed = bool(item.get("refill_needed", False))

            if not refill_needed:
                continue

            pub = self.refill_flag_pubs.get(medicine_name)
            if pub is None:
                self.get_logger().warn(
                    f"Bool 리필 토픽 매핑이 없는 약입니다: {medicine_name}"
                )
                continue

            self._publish_bool(pub, True)

    def _publish_refill_requests(self, task: dict):
        refill_items = [
            item
            for item in task["items"]
            if bool(item.get("refill_needed", False))
        ]

        for item in refill_items:
            msg = String()
            msg.data = json.dumps(
                {
                    "event_id": task["event_id"],
                    "prescription_name": task.get("prescription_name"),
                    **item,
                },
                ensure_ascii=False,
            )
            self.refill_request_pub.publish(msg)

        if refill_items:
            self.get_logger().info(f"리필 요청 publish: {len(refill_items)}개")

    def _publish_refill_needed(self, items: list):
        refill_needed = any(
            bool(item.get("refill_needed", False))
            for item in items
        )

        self._publish_bool(self.refill_needed_pub, refill_needed)

        if refill_needed:
            self.get_logger().info("약품 리필 필요 여부 publish: True")
        else:
            self.get_logger().info("약품 리필 필요 여부 publish: False, 처방 시작 가능")

    def _publish_bool(self, publisher, value: bool):
        msg = Bool()
        msg.data = value
        publisher.publish(msg)

    def _publish_task_state(self, event_id, status_value: str, message: str):
        msg = String()
        msg.data = json.dumps(
            {
                "event_id": event_id,
                "status": status_value,
                "message": message,
            },
            ensure_ascii=False,
        )
        self.task_state_pub.publish(msg)

    def _get_json(self, path: str):
        url = f"{self.backend_base_url}{path}"
        headers = {"Accept": "application/json"}

        req = request.Request(url, headers=headers, method="GET")

        try:
            with request.urlopen(req, timeout=self.request_timeout_sec) as response:
                raw = response.read().decode("utf-8")
                return response.status, self._decode_json(raw)
        except error.HTTPError as e:
            raw = e.read().decode("utf-8")
            return e.code, self._decode_json(raw)
        except error.URLError as e:
            self.get_logger().error(f"백엔드 연결 실패: {url} ({e})")
            return 0, {"error": str(e)}
        except TimeoutError as e:
            self.get_logger().error(f"백엔드 요청 timeout: {url} ({e})")
            return 0, {"error": str(e)}

    def _decode_json(self, raw: str):
        if not raw:
            return {}

        try:
            return json.loads(raw)
        except json.JSONDecodeError:
            return {"raw": raw}


def main(args=None):
    rclpy.init(args=args)
    node = Task_Manager()

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

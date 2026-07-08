# task_manager_bridge.py

# get 요청을 3군데 로 보낸거 (브릿지가 get을 db에 요청 > 정보를 받는다)
# 1. event(처방전 발생)가 발생했을 때 정보
# 2. medicine 테이블에 있는 정보
# 3. 리필 여부 상태 정보
# > task manager에 pub
# ---
# task manager 판단
# ~~1. 약품이 필요한가?~~ 
# 1) 약이름, 약 좌표(적재소, 조제기), order 번호 <--- 리필 필
# 2) 약이름, 약 좌표 (조제기), order 번호 <----- 리필 불필

# 필요한 개수>조제기 : 그냥 스크래퍼 로 이동 명령



import json
from urllib import error, request

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


DEFAULT_BACKEND_BASE_URL = "http://172.23.0.128:8000/api"
DEFAULT_EVENTS_TOPIC = "dsr01/pharmacy/events"
DEFAULT_MEDICINE_TOPIC = "dsr01/pharmacy/medicine"


class TaskManagerBridge(Node):
    def __init__(self):
        super().__init__("task_manager_bridge")

        self.declare_parameter("backend_base_url", DEFAULT_BACKEND_BASE_URL)
        self.declare_parameter("poll_interval_sec", 1.0)
        self.declare_parameter("request_timeout_sec", 2.0)
        self.declare_parameter("events_topic", DEFAULT_EVENTS_TOPIC)
        self.declare_parameter("medicine_topic", DEFAULT_MEDICINE_TOPIC)

        self.backend_base_url = (
            self.get_parameter("backend_base_url")
            .get_parameter_value()
            .string_value
            .rstrip("/")
        )
        self.poll_interval_sec = (
            self.get_parameter("poll_interval_sec").get_parameter_value().double_value
        )
        self.request_timeout_sec = (
            self.get_parameter("request_timeout_sec").get_parameter_value().double_value
        )
        self.events_topic = (
            self.get_parameter("events_topic").get_parameter_value().string_value
        )
        self.medicine_topic = (
            self.get_parameter("medicine_topic").get_parameter_value().string_value
        )

        self.events_pub = self.create_publisher(String, self.events_topic, 10)
        self.medicine_pub = self.create_publisher(String, self.medicine_topic, 10)

        self._logged_success = set()
        self._logged_errors = set()

        self.create_timer(self.poll_interval_sec, self._poll_backend)

        self.get_logger().info(
            f"task_manager_bridge started. backend={self.backend_base_url}, "
            f"events_topic={self.events_topic}, medicine_topic={self.medicine_topic}"
        )

    def _poll_backend(self):
        self._get_and_publish("events", "/events/", self.events_pub)
        self._get_and_publish("medicine", "/medicine/", self.medicine_pub)

    def _get_and_publish(self, label: str, path: str, publisher):
        status_code, data = self._get_json(path)

        if status_code != 200:
            error_key = (label, status_code, json.dumps(data, ensure_ascii=False))
            if error_key not in self._logged_errors:
                self.get_logger().warn(f"{label} GET 실패: {status_code} {data}")
                self._logged_errors.add(error_key)
            return

        msg = String()
        msg.data = json.dumps(data, ensure_ascii=False)
        publisher.publish(msg)

        if label not in self._logged_success:
            self.get_logger().info(
                f"{label} GET 성공 및 publish 완료: {self._count_items(data)}개"
            )
            self._logged_success.add(label)

    def _get_json(self, path: str):
        url = f"{self.backend_base_url}{path}"
        req = request.Request(
            url,
            headers={"Accept": "application/json"},
            method="GET",
        )

        try:
            with request.urlopen(req, timeout=self.request_timeout_sec) as response:
                raw = response.read().decode("utf-8")
                return response.status, self._decode_json(raw)
        except error.HTTPError as e:
            raw = e.read().decode("utf-8")
            return e.code, self._decode_json(raw)
        except error.URLError as e:
            return 0, {"error": str(e)}
        except TimeoutError as e:
            return 0, {"error": str(e)}

    def _decode_json(self, raw: str):
        if not raw:
            return {}

        try:
            return json.loads(raw)
        except json.JSONDecodeError:
            return {"raw": raw}

    def _count_items(self, data) -> int:
        if isinstance(data, list):
            return len(data)
        if isinstance(data, dict):
            results = data.get("results")
            if isinstance(results, list):
                return len(results)
        return 1


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

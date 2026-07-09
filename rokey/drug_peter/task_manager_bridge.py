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


# 백엔드 API 기본 주소.
# 다른 PC에서 백엔드가 실행 중이면 실행 시 파라미터로 바꿀 수 있다.
DEFAULT_BACKEND_BASE_URL = "http://172.23.0.128:8000/api"

# 브릿지가 publish할 기본 ROS 토픽 이름..
# /api/events/ 응답 전체는 DEFAULT_EVENTS_TOPIC으로 나가고,
# /api/medicine/ 응답 전체는 DEFAULT_MEDICINE_TOPIC으로 나간다.
DEFAULT_EVENTS_TOPIC = "dsr01/pharmacy/events"
DEFAULT_MEDICINE_TOPIC = "dsr01/pharmacy/medicine"


class TaskManagerBridge(Node):
    def __init__(self):
        # ROS2 노드 이름.
        super().__init__("task_manager_bridge")
        self.get_logger().info("task_manager_bridge 노드 실행...")
        # ROS2 파라미터 선언.
        # 예:
        # ros2 run rokey task_manager_bridge --ros-args \
        #   -p backend_base_url:=http://172.23.0.128:8000/api
        self.declare_parameter("backend_base_url", DEFAULT_BACKEND_BASE_URL)
        self.declare_parameter("poll_interval_sec", 1.0)
        self.declare_parameter("request_timeout_sec", 2.0)
        self.declare_parameter("events_topic", DEFAULT_EVENTS_TOPIC)
        self.declare_parameter("medicine_topic", DEFAULT_MEDICINE_TOPIC)

        # URL 뒤에 /가 붙어 들어와도 /events/와 합칠 때 //가 생기지 않게 제거한다.
        self.backend_base_url = (
            self.get_parameter("backend_base_url")
            .get_parameter_value()
            .string_value
            .rstrip("/")
        )

        # 백엔드 정보를 몇 초마다 다시 가져올지 정한다.
        self.poll_interval_sec = (
            self.get_parameter("poll_interval_sec").get_parameter_value().double_value
        )

        # 백엔드가 응답하지 않을 때 최대 몇 초까지 기다릴지 정한다.
        self.request_timeout_sec = (
            self.get_parameter("request_timeout_sec").get_parameter_value().double_value
        )

        # publish할 토픽 이름도 파라미터로 바꿀 수 있게 한다.
        self.events_topic = (
            self.get_parameter("events_topic").get_parameter_value().string_value
        )
        self.medicine_topic = (
            self.get_parameter("medicine_topic").get_parameter_value().string_value
        )

        # 백엔드에서 받은 JSON을 std_msgs/String의 data에 담아 publish한다.
        # 받는 노드는 json.loads(msg.data)로 다시 Python dict/list로 바꿔 쓰면 된다.
        self.events_pub = self.create_publisher(String, self.events_topic, 10)
        self.medicine_pub = self.create_publisher(String, self.medicine_topic, 10)

        # 같은 로그가 매초 반복되면 보기 힘드니 첫 성공/첫 에러만 찍기 위한 기록.
        self._logged_success = set()
        self._logged_errors = set()

        # 이 노드는 subscription 없이 timer로 백엔드를 polling한다.
        self.create_timer(self.poll_interval_sec, self._poll_backend)

        self.get_logger().info(
            f"task_manager_bridge started. backend={self.backend_base_url}, "
            f"events_topic={self.events_topic}, medicine_topic={self.medicine_topic}"
        )

    def _poll_backend(self):
        # db_______.txt 기준으로 이 브릿지는 GET 두 개만 수행한다.
        # DB를 수정하는 POST/PUT/PATCH/DELETE 요청은 하지 않는다.
        self._get_and_publish("events", "/events/", self.events_pub)
        self._get_and_publish("medicine", "/medicine/", self.medicine_pub)

    def _get_and_publish(self, label: str, path: str, publisher):
        # path에 해당하는 백엔드 API를 GET으로 읽어온다.
        status_code, data = self._get_json(path)

        # 200 OK가 아니면 publish하지 않고 경고 로그만 남긴다.
        if status_code != 200:
            error_key = (label, status_code, json.dumps(data, ensure_ascii=False))
            if error_key not in self._logged_errors:
                self.get_logger().warn(f"{label} GET 실패: {status_code} {data}")
                self._logged_errors.add(error_key)
            return

        # Python dict/list 데이터를 ROS String 메시지에 넣을 수 있게 JSON 문자열로 변환한다.
        # ensure_ascii=False를 쓰면 한글 약 이름이 그대로 보인다.
        msg = String()
        msg.data = json.dumps(data, ensure_ascii=False)
        publisher.publish(msg)

        # 성공 로그는 첫 publish 때만 출력한다. publish 자체는 timer마다 계속 수행된다.
        if label not in self._logged_success:
            self.get_logger().info(
                f"{label} GET 성공 및 publish 완료: {self._count_items(data)}개"
            )
            self._logged_success.add(label)

    def _get_json(self, path: str):
        # 예: http://172.23.0.128:8000/api + /events/
        #   -> http://172.23.0.128:8000/api/events/
        url = f"{self.backend_base_url}{path}"

        # 이 브릿지는 읽기 전용이므로 HTTP method는 항상 GET이다.
        # data/body를 넣지 않아서 POST 요청이 발생하지 않는다.
        req = request.Request(
            url,
            headers={"Accept": "application/json"},
            method="GET",
        )

        try:
            # 실제 HTTP 요청을 보내고, 응답 body를 UTF-8 문자열로 읽는다.
            with request.urlopen(req, timeout=self.request_timeout_sec) as response:
                raw = response.read().decode("utf-8")
                return response.status, self._decode_json(raw)
        except error.HTTPError as e:
            # 백엔드가 404/500 같은 HTTP 에러를 반환한 경우.
            # 에러 body도 JSON일 수 있으니 decode해서 위쪽으로 넘긴다.
            raw = e.read().decode("utf-8")
            return e.code, self._decode_json(raw)
        except error.URLError as e:
            # IP/포트가 틀렸거나 백엔드가 꺼져 있거나 네트워크가 닿지 않는 경우.
            return 0, {"error": str(e)}
        except TimeoutError as e:
            # request_timeout_sec 안에 응답이 오지 않은 경우.
            return 0, {"error": str(e)}

    def _decode_json(self, raw: str):
        # body가 비어 있으면 빈 dict로 통일한다.
        if not raw:
            return {}

        try:
            return json.loads(raw)
        except json.JSONDecodeError:
            # 백엔드가 JSON이 아닌 문자열/HTML을 반환해도 노드가 죽지 않게 한다.
            return {"raw": raw}

    def _count_items(self, data) -> int:
        # 로그용 개수 계산.
        # DRF pagination이 꺼져 있으면 list가 오고,
        # 켜져 있으면 {"results": [...]} 형태가 올 수 있어 둘 다 처리한다.
        if isinstance(data, list):
            return len(data)
        if isinstance(data, dict):
            results = data.get("results")
            if isinstance(results, list):
                return len(results)
        return 1


def main(args=None):
    # ROS2 Python 클라이언트 초기화.
    rclpy.init(args=args)

    # 브릿지 노드 생성.
    node = TaskManagerBridge()
    
    try:
        # timer callback이 계속 실행되도록 spin한다.
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Ctrl+C 종료 처리.
        pass
    finally:
        # 노드와 ROS2 리소스 정리.
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

# scraper_packaging_node.py
# 담당: 박현정
# 역할: 스크래퍼로 조제된 약을 받아 종이봉투에 붓기
#
# 두 단계 신호로 나뉜다 (한 번에 다 실행하는 게 아님):
# 각 단계는 "신호를 받으면" 실행된다. 신호가 원인(트리거)이고,
# 아래 상태 확인은 "지금 실행해도 안전한 상태인지"를 걸러주는 안전장치일 뿐,
# 상태 그 자체가 동작을 일으키는 게 아니다.
#
#   [/dsr01/pharmacy/events 수신 -> 이게 트리거] 1~3번 실행 후 "대기 상태"로 들어감
#     1. 스크래퍼 거치대로 이동
#     2. 스크래퍼 손잡이 잡기
#     3. 약 나오는 곳(조제기 배출구)에 스크래퍼 대고 대기
#
#   [/dsr01/pharmacy/events 수신 -> 이게 트리거] 4~5번 실행
#     (단, "대기 상태"일 때만 -> 안 그러면 무시. 예: 아직 스크래퍼도 안 든 상태에서
#      신호가 잘못 오거나, 이미 처리 중인데 신호가 중복으로 오는 경우를 걸러냄)
#     4. 스크래퍼(약 든 채로) 들고 봉투 위치로 이동
#     5. 비스듬히 기울여서 붓기 -> 거치대로 복귀 + 놓기

import json
import sys

import rclpy
import DR_init
from time import sleep
from std_msgs.msg import String

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 50, 50          # movej(관절 이동) 속도 - 관절 회전 빠르기
LINEAR_V, LINEAR_A = 60, 60   # movel(직선 이동) 속도 - 직선 이동 빠르기

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

ON, OFF = 1, 0

# ---- TCP 프로필 ----
TCP_GRIPPER_ONLY = "Tool_v1"    # 빈손(그리퍼만) 기준 - 거치대, 스크래퍼 잡기 관련 동작용
TCP_WITH_SCRAPER = "Tool_scraper"    # 그리퍼+스크래퍼 합친 기준 - 스크래퍼 잡았을 때 관련 동작용

# ---- 좌표 ----
HOME = [0, 0, 90.00, 0, 90.00, 0] 

TOOL_STAND_SCRAPER = [16.97, 17.76, 95.13, -197.14, -36.27, 37.03]
DISPENSING_POINT = [-30.39, 4.17, 118.46, -120.53, -27.26, -75.17]
POUCH_POS_ABOVE = [9.13, 27.12, 89.56, -46.16, 79.67, -61.32]
POUCH_POS = [-0.23, 20.16, 89.98, -36.90, 74.35, -74.70]

# 상태값
STATE_IDLE = "idle"                      # 아무 작업도 안 하는 중
STATE_WAITING_AT_DISPENSER = "waiting"   # 조제기에서 약 받으며 대기 중 (검수완료 기다림)


class ScraperPackagingNode:
    def __init__(self, node):
        self.node = node
        self.state = STATE_IDLE

        # 토픽으로부터 받는 status 값을 우리가 직접 보고 "픽업 시작할 때"인지 "붓기 시작할 때"인지 판단한다.
        self.task_event_sub = node.create_subscription(
            String, '/dsr01/pharmacy/events', self.on_task_event, 10
        )

    # ------------------------------------------------------------
    # 이벤트 상태에 따라 분기: READY -> 픽업 시작, DONE -> 붓기 시작
    # status 값이 원인(트리거)이고, self.state 체크는 안전장치일 뿐이다.
    # ------------------------------------------------------------

    def on_task_event(self, msg: String):
        payload = self._parse(msg)
        if payload is None:
            return

        status = payload.get("status")
        patient_name = payload.get("prescription_name", "이름없음")
        med_names = ", ".join(
            item.get("medicine_name", "?") for item in payload.get("items", [])
        )

        # items가 비어있으면 all()이 True로 착각할 수 있어서 빈 리스트는 명시적으로 제외
        all_items_ready = bool(items) and all(
            item.get("status") == "READY" for item in items
        )

        if outer_status == "DONE":
            self._start_pour(payload, patient_name, med_names)
        elif all_items_ready:
            self._start_pickup(payload, patient_name, med_names)
        else:
            item_statuses = [item.get("status") for item in items]
            self.node.get_logger().info(
                f"아직 트리거 조건 안 됨 (outer_status={outer_status}, "
                f"item_statuses={item_statuses}) - 무시"
            )

    def _start_pickup(self, payload, patient_name, med_names):
        # 안전장치: 이미 다른 처방 처리 중이면 무시
        if self.state != STATE_IDLE:
            self.node.get_logger().warn(
                f"이미 다른 작업 처리 중(state={self.state}), 시작 신호 무시: {payload}"
            )
            return

        try:
            self.node.get_logger().info(
                f"[시작 신호] patient={patient_name}, med={med_names} -> 스크래퍼 픽업 시작"
            )
            self.pickup_and_wait()
            self.state = STATE_WAITING_AT_DISPENSER
            self.node.get_logger().info(
                "배출구에서 대기 중 - 검수완료(status=DONE) 기다림"
            )
        except Exception as e:
            self.node.get_logger().error(f"pickup_and_wait 실패: {e}")
            self.state = STATE_IDLE


    def _start_pour(self, payload, patient_name, med_names):
        # 안전장치: 배출구 대기 상태가 아니면(스크래퍼도 안 들었거나, 이미 처리 중이면) 무시
        if self.state != STATE_WAITING_AT_DISPENSER:
            self.node.get_logger().warn(
                f"배출구 대기 상태가 아님(state={self.state}), 완료 신호 무시: {payload}"
            )
            return

        try:
            self.node.get_logger().info(f"[완료 신호] patient={patient_name} -> 붓기 시작")
            self.pour_and_return()
        except Exception as e:
            self.node.get_logger().error(f"pour_and_return 실패: {e}")
        finally:
            self.state = STATE_IDLE
            self.node.get_logger().info("사이클 종료, idle 상태로 복귀")

    def pickup_and_wait(self):
        node = self.node
        # 0. 항상 HOME에서 시작 (이전 사이클 종료 위치와 무관하게 항상 같은 조건에서 출발)
        node.get_logger().info("[0/3] moving to home")
        movej(posj(*HOME), vel=VELOCITY, acc=ACC)

        self.release()

        # 1. 스크래퍼 tool 거치대로 이동 + tool gripping
        node.get_logger().info("[1/3] 거치대 위로 이동")
        movej(posj(*TOOL_STAND_SCRAPER), vel=VELOCITY, acc=ACC)
        # 2. (grip 포함, 제조기로 이동)
        self.grip()

        node.get_logger().info("[2/3] 스크래퍼 잡음")

        # ---- 스크래퍼를 실제로 쥔 순간부터는 TCP를 스크래퍼 기준으로 전환 ----
        set_tcp(TCP_WITH_SCRAPER)
        node.get_logger().info(f"TCP 전환: {TCP_WITH_SCRAPER}")

        node.get_logger().info("제조기로 이동중")

        movel(
            posx(-10, 0, 0, 0, 0, 0),
            mod=DR_MV_MOD_REL,
            ref=DR_BASE,
            v=LINEAR_V,
            a=LINEAR_A
        )

        movel(
            posx(0, 0, 160, 0, 0, 0),
            mod=DR_MV_MOD_REL,
            ref=DR_BASE,
            v=LINEAR_V,
            a=LINEAR_A
        )

        movel(
            posx(-150, 0, 0, 0, 0, 0),
            mod=DR_MV_MOD_REL,
            ref=DR_BASE,
            v=LINEAR_V,
            a=LINEAR_A
        )

        movel(
            posx(0, -250, 0, 0, 0, 0),
            mod=DR_MV_MOD_REL,
            ref=DR_BASE,
            v=LINEAR_V,
            a=LINEAR_A
        )

        # 3. 약 나오는 곳으로 이동해서 대기
        node.get_logger().info("[3/3] 제조기로 이동하여 대기")
        movej(posj(*DISPENSING_POINT), vel=VELOCITY, acc=ACC)


    def pour_and_return(self):
        node = self.node

        # 4. 약이 스크래퍼에 담긴 상태로 봉투 위치까지 이동
        node.get_logger().info("[4/5] 약이 스크래퍼에 담긴 상태로 봉투 위치까지 이동")
        movej(posj(*POUCH_POS_ABOVE), vel=VELOCITY, acc=ACC)

        # 5. 기울여서 봉투에 붓기
        node.get_logger().info("[5/5] tilting to pour into pouch")
        movej(posj(*POUCH_POS), vel=VELOCITY, acc=ACC)
        sleep(0.5)  # 약이 다 쏟아지도록 잠깐 대기 (실측 후 조정)

 
 
        # 봉투 위치에서 벗어나 tool 거치대로 복귀 + 스크래퍼 반납
        # node.get_logger().info("툴 거치대로 이동하기 위해 중앙으로 복귀")
        # movej(posj(-2.58, 3.88, 80.00, 358.00, 96.40, -358.00), vel=VELOCITY, acc=ACC) # 스크래퍼 TCP 기준으로 다시 체크 할 것 이동범위가 너무 큼
        movej(posj(11.03, 1.16, 89.41, -0.92, 90.33, -79.64), vel=VELOCITY, acc=ACC)
        sleep(0.5)
        node.get_logger().info("툴 거치대로 이동 중")
        movej(posj(17.76, 24.26, 82.79, -12.23, 52.64, -154.53), vel=VELOCITY, acc=ACC)
        
        self.release()
        movel(
            posx(-20, 0, 0, 0, 0, 0),
            mod=DR_MV_MOD_REL,
            ref=DR_BASE,
            v=LINEAR_V,
            a=LINEAR_A
        )
        self.grip()
        set_tcp(TCP_GRIPPER_ONLY)
        node.get_logger().info(f"TCP 전환: {TCP_GRIPPER_ONLY} (거치대 복귀후")

        # 중앙 HOME 으로 이동
        movej(posj(0,0,90,0,90,0), vel=VELOCITY, acc=ACC)

    # ------------------------------------------------------------
    def grip(self):
        self.node.get_logger().info("set for digital output 0 1 for grip")
        set_digital_output(1, OFF)
        set_digital_output(2, OFF)
        set_digital_output(1, OFF)
        set_digital_output(2, ON)
        sleep(1)
 
    def release(self):
        self.node.get_logger().info("set for digital output 1 0 for release")
        set_digital_output(1, OFF)
        set_digital_output(2, OFF)
        set_digital_output(1, ON)
        set_digital_output(2, ON)
        sleep(1)

    def _parse(self, msg: String):
        try:
            return json.loads(msg.data)
        except json.JSONDecodeError:
            self.node.get_logger().warn(f"invalid JSON, ignoring: {msg.data}")
            return None


def main(args=None):
    global movej, movel, set_tool, set_tcp
    global set_digital_output, get_digital_input
    global DR_BASE, DR_MV_MOD_ABS, DR_MV_MOD_REL
    global posj, posx
    global wait

    rclpy.init(args=args)
    node = rclpy.create_node("scraper_packaging_node", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            movej, movel, set_tool, set_tcp,
            set_digital_output, get_digital_input,
            DR_BASE, DR_MV_MOD_ABS, DR_MV_MOD_REL,
            wait
        )
        from DR_common2 import posj, posx
    except ImportError as e:
        node.get_logger().error(f"Error importing DSR_ROBOT2: {e}")
        return

    set_tool("Tool Weight_1")
    set_tcp(TCP_GRIPPER_ONLY)  # 시작은 항상 빈손 상태이므로 그리퍼 전용 TCP로

    logic = ScraperPackagingNode(node)

    # ------------------------------------------------------------
    # 테스트 편의용: `python3 scraper_packaging_node.py --test` 로 실행하면
    # ros2 topic pub 없이도 노드 뜨자마자 자동으로 task_start를 흉내내서 보낸다.
    # 실제 운영 때는 --test 없이 실행하여 신호를 기다린다.
    # ------------------------------------------------------------
    if "--test" in sys.argv:
        node.get_logger().info("[TEST MODE] status WAITING -> DONE 자동 연속 실행 (1~5번)")

        fake_waiting_msg = String()
        fake_waiting_msg.data = json.dumps({
            "id": 999,
            "prescription_name": "테스트 김철수",
            "status": "WAITING",
            "items": [{"medicine_name": "부루펜", "quantity": 1, "order": 1}],
        }, ensure_ascii=False)
        logic.on_task_event(fake_waiting_msg)  # 1~3번 실행, 끝나면 state=WAITING

        fake_done_msg = String()
        fake_done_msg.data = json.dumps({
            "id": 999,
            "prescription_name": "테스트 김철수",
            "status": "DONE",
            "items": [{"medicine_name": "부루펜", "quantity": 1, "order": 1}],
        }, ensure_ascii=False)
        logic.on_task_event(fake_done_msg)  # 이어서 바로 4~5번까지 실행

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
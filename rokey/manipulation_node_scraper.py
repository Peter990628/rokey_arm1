# scraper_packaging_node.py
# 담당: 박현정
# 역할: 스크래퍼 tool로 조제된 약을 받아 종이봉투에 붓기

import rclpy
import DR_init
from time import sleep
from std_msgs.msg import Bool

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 30, 30

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

ON, OFF = 1, 0

# ---- 좌표 (실측 후 티칭값으로 교체 필요) ----
TOOL_STAND_SCRAPER = [0, 0, 0, 0, 0, 0]   # 스크래퍼 tool 거치대 위 (approach)
TOOL_STAND_SCRAPER_DOWN = [0, 0, -50, 0, 0, 0]  # 실제 파지 위치 (relative)
POUCH_POS = [0, 0, 0, 0, 0, 0]            # 종이봉투 위치 (해벽님이 거치한 위치로 교체)
POUCH_POS_ABOVE = [0, 0, 50, 0, 0, 0]     # 봉투 위 안전 접근 높이


class ScraperPackagingNode:
    def __init__(self, node, robot_api):
        self.node = node
        self.api = robot_api  # movel, movej, posx 등 DSR_ROBOT2 함수 묶음
        self.busy = False

        self.sub = node.create_subscription(
            Bool, '/pharmacy/med_ready', self.on_med_ready, 10)

        self.done_pub = node.create_publisher(
            Bool, '/pharmacy/packaging_ready', 10)

        node.get_logger().info("scraper_packaging_node ready. waiting for /pharmacy/med_ready")

    def on_med_ready(self, msg: Bool):
        if not msg.data:
            return
        if self.busy:
            self.node.get_logger().warn("already processing, ignoring duplicate trigger")
            return

        self.busy = True
        try:
            self.run_sequence()
        except Exception as e:
            self.node.get_logger().error(f"sequence failed: {e}")
        finally:
            self.busy = False

    def run_sequence(self):
        api = self.api
        node = self.node

        # 1. 스크래퍼 tool 거치대로 이동 + tool gripping
        node.get_logger().info("[1/5] moving to scraper tool stand")
        api.movel(api.posx(*TOOL_STAND_SCRAPER), mod=api.DR_MV_MOD_ABS,
                  ref=api.DR_BASE, radius=70, v=800, a=800)
        api.movel(api.posx(*TOOL_STAND_SCRAPER_DOWN), mod=api.DR_MV_MOD_REL,
                  radius=0, v=300, a=300)
        self.grip()
        api.movel(api.posx(0, 0, 50, 0, 0, 0), mod=api.DR_MV_MOD_REL,
                  radius=0, v=500, a=500)

        # 2. 약이 스크래퍼에 담긴 상태로 봉투 위치까지 이동
        node.get_logger().info("[2/5] moving to pouch position")
        api.movel(api.posx(*POUCH_POS_ABOVE), mod=api.DR_MV_MOD_ABS,
                  ref=api.DR_BASE, radius=70, v=800, a=800)
        api.movel(api.posx(*POUCH_POS), mod=api.DR_MV_MOD_ABS,
                  ref=api.DR_BASE, radius=0, v=300, a=300)

        # 3. 기울여서 봉투에 붓기
        node.get_logger().info("[3/5] tilting to pour into pouch")
        api.movel(api.posx(0, 0, 0, 0, 60, 0), mod=api.DR_MV_MOD_REL,
                  radius=0, v=200, a=200)
        sleep(1.0)  # 약이 다 쏟아지도록 잠깐 대기 (실측 후 조정)
        api.movel(api.posx(0, 0, 0, 0, -60, 0), mod=api.DR_MV_MOD_REL,
                  radius=0, v=200, a=200)

        # 4. 봉투 위치에서 벗어나 tool 거치대로 복귀
        node.get_logger().info("[4/5] returning scraper to tool stand")
        api.movel(api.posx(0, 0, 50, 0, 0, 0), mod=api.DR_MV_MOD_REL,
                  radius=0, v=500, a=500)
        api.movel(api.posx(*TOOL_STAND_SCRAPER), mod=api.DR_MV_MOD_ABS,
                  ref=api.DR_BASE, radius=70, v=800, a=800)
        api.movel(api.posx(*TOOL_STAND_SCRAPER_DOWN), mod=api.DR_MV_MOD_REL,
                  radius=0, v=300, a=300)
        self.release()
        api.movel(api.posx(0, 0, 50, 0, 0, 0), mod=api.DR_MV_MOD_REL,
                  radius=0, v=500, a=500)

        # 5. 완료 상태 pub → 조해벽 노드로 전달
        node.get_logger().info("[5/5] packaging step done, publishing signal")
        self.done_pub.publish(Bool(data=True))

    def grip(self):
        self.api.set_digital_output(1, OFF)
        self.api.set_digital_output(2, ON)
        sleep(1)

    def release(self):
        self.api.set_digital_output(1, ON)
        self.api.set_digital_output(2, OFF)
        sleep(1)


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("scraper_packaging_node", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            movej, movel, set_tool, set_tcp,
            set_digital_output, get_digital_input,
            DR_BASE, DR_MV_MOD_ABS, DR_MV_MOD_REL,
        )
        from DR_common2 import posj, posx
    except ImportError as e:
        node.get_logger().error(f"Error importing DSR_ROBOT2: {e}")
        return

    set_tool("Tool Weight_1")
    set_tcp("GripperDA_v1")

    # api 객체처럼 묶어서 노드 클래스에 전달 (테스트 편의를 위해 네임스페이스 객체화)
    class Api:
        pass
    api = Api()
    api.movel = movel
    api.movej = movej
    api.posx = posx
    api.posj = posj
    api.DR_BASE = DR_BASE
    api.DR_MV_MOD_ABS = DR_MV_MOD_ABS
    api.DR_MV_MOD_REL = DR_MV_MOD_REL
    api.set_digital_output = set_digital_output

    logic = ScraperPackagingNode(node, api)

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
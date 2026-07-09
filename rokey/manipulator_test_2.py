# manipulator_test_2.py
# paper_bag_test + manipulation_node_scraper
#
# 역할:
# 1. 스크래퍼를 집어서 조제기 배출구로 이동
# 2. 스크래퍼에 담긴 약을 종이봉투에 붓고 스크래퍼 반납
# 3. 종이봉투를 잡아서 수납대 위치에 내려놓기
# 4. 전체 테스트 동작 완료 후 task_done pub

import sys
from time import sleep

import rclpy
import DR_init
from std_msgs.msg import Bool


ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

ON, OFF = 1, 0

# ---- 속도 ----
PAPER_VELOCITY, PAPER_ACC = 30, 30
SCRAPER_VELOCITY, SCRAPER_ACC = 50, 50
SCRAPER_LINEAR_V, SCRAPER_LINEAR_A = 60, 60

# ---- TCP 프로필 ----
TCP_GRIPPER_ONLY = "Tool_v1"
TCP_WITH_SCRAPER = "Tool_scraper"

# ---- 공통 위치 ----
HOME = [0, 0, 90, 0, 90, 0]

# ---- 종이봉투 위치 ----
PAPER_BAG_ABOVE = [-36.32, 53.19, 28.79, -0.59, 98.47, -130.76]
PAPER_BAG_GRIP = [-37.34, 57.57, 46.94, -0.61, 75.93, -131.56]
PAPER_BAG_Y_AWAY = [-79.23, 48.44, 36.97, -0.07, 94.56, -42.92]
PAPER_BAG_SHELF = [-51.77, -8.32, 84.92, -0.72, 101.25, -52.54]

# ---- 스크래퍼 위치 ----
TOOL_STAND_SCRAPER = [16.97, 17.76, 95.13, -197.14, -36.27, 37.03]
DISPENSING_POINT = [-30.39, 4.17, 118.46, -120.53, -27.26, -75.17]
POUCH_POS_ABOVE = [9.13, 27.12, 89.56, -46.16, 79.67, -61.32]
POUCH_POS = [-0.23, 20.16, 89.98, -36.90, 74.35, -74.70]
SCRAPER_RETURN_MIDDLE = [11.03, 1.16, 89.41, -0.92, 90.33, -79.64]
SCRAPER_RETURN_STAND = [17.76, 24.26, 82.79, -12.23, 52.64, -154.53]


class ManipulatorTest2:
    def __init__(self, node):
        self.node = node
        self.task_done_pub = node.create_publisher(Bool, "task_done", 10)

    # ------------------------------------------------------------
    # gripper 동작
    # ------------------------------------------------------------
    def grip(self):
        self.node.get_logger().info("grip: digital output 0 1")
        set_digital_output(1, OFF)
        set_digital_output(2, OFF)
        set_digital_output(1, OFF)
        set_digital_output(2, ON)
        sleep(1)

    def release(self):
        # manipulation_node_scraper.py 의 release() 동작 그대로 사용
        self.node.get_logger().info("release: digital output 1 1")
        set_digital_output(1, OFF)
        set_digital_output(2, OFF)
        set_digital_output(1, ON)
        set_digital_output(2, ON)
        sleep(1)

    # ------------------------------------------------------------
    # 1단계: 종이봉투를 잡아서 수납대 위에 내려놓기
    # ------------------------------------------------------------
    def run_paper_bag_sequence(self) -> bool:
        self.node.get_logger().info("[PAPER 0/7] 종이봉투 작업 시작")

        set_tcp(TCP_GRIPPER_ONLY)
        self.release()

        self.node.get_logger().info("[PAPER 1/7] 홈으로 이동")
        movej(posj(*HOME), vel=PAPER_VELOCITY, acc=PAPER_ACC)
        wait(0.5)

        self.node.get_logger().info("[PAPER 2/7] 봉투 위로 이동")
        movej(posj(*PAPER_BAG_ABOVE), vel=PAPER_VELOCITY, acc=PAPER_ACC)
        wait(0.5)

        self.node.get_logger().info("[PAPER 3/7] 봉투 위치로 내려가서 잡기")
        movej(posj(*PAPER_BAG_GRIP), vel=PAPER_VELOCITY, acc=PAPER_ACC)
        self.grip()

        self.node.get_logger().info("[PAPER 4/7] 봉투를 아래로 살짝 빼기")
        wait(0.5)
        movel(
            posx(0, 0, -100, 0, 0, 0),
            mod=DR_MV_MOD_REL,
            ref=DR_BASE,
            v=100,
            a=50,
        )

        self.node.get_logger().info("[PAPER 5/7] 봉투를 위로 들어 올리기")
        wait(0.5)
        movel(
            posx(0, -50, 300, 0, 0, 0),
            mod=DR_MV_MOD_REL,
            ref=DR_BASE,
            v=50,
            a=50,
        )

        self.node.get_logger().info("[PAPER 6/7] 수납대 이동 전 간섭 회피 위치로 이동")
        wait(0.5)
        movej(posj(*PAPER_BAG_Y_AWAY), vel=PAPER_VELOCITY, acc=PAPER_ACC)

        self.node.get_logger().info("[PAPER 7/7] 수납대 위로 이동")
        wait(0.5)
        movej(posj(*PAPER_BAG_SHELF), vel=PAPER_VELOCITY, acc=PAPER_ACC)

        current_posj = get_current_posj()
        if self._is_at_paper_bag_shelf(current_posj):
            self.release()
            self.node.get_logger().info("종이봉투 수납대 배치 완료")
            return True

        self.node.get_logger().warn(
            f"수납대 목표 위치 확인 실패. current_posj={current_posj}"
        )
        return False

    def _is_at_paper_bag_shelf(self, current_posj) -> bool:
        return (
            -52.00 <= current_posj[0] <= -50.00
            and -9.00 <= current_posj[1] <= -8.00
            and 84.00 <= current_posj[2] <= 86.00
        )

    # ------------------------------------------------------------
    # 2단계: 스크래퍼를 집어서 조제기 배출구에서 대기
    # ------------------------------------------------------------
    def pickup_scraper_and_wait(self):
        self.node.get_logger().info("[SCRAPER 0/3] 스크래퍼 픽업 시작")

        set_tcp(TCP_GRIPPER_ONLY)

        self.node.get_logger().info("[SCRAPER 1/3] 홈으로 이동")
        movej(posj(*HOME), vel=SCRAPER_VELOCITY, acc=SCRAPER_ACC)
        self.release()

        self.node.get_logger().info("[SCRAPER 2/3] 스크래퍼 거치대로 이동 후 잡기")
        movej(posj(*TOOL_STAND_SCRAPER), vel=SCRAPER_VELOCITY, acc=SCRAPER_ACC)
        self.grip()

        set_tcp(TCP_WITH_SCRAPER)
        self.node.get_logger().info(f"TCP 전환: {TCP_WITH_SCRAPER}")

        self.node.get_logger().info("[SCRAPER 3/3] 조제기 배출구로 이동")
        movel(
            posx(-10, 0, 0, 0, 0, 0),
            mod=DR_MV_MOD_REL,
            ref=DR_BASE,
            v=SCRAPER_LINEAR_V,
            a=SCRAPER_LINEAR_A,
        )
        movel(
            posx(0, 0, 160, 0, 0, 0),
            mod=DR_MV_MOD_REL,
            ref=DR_BASE,
            v=SCRAPER_LINEAR_V,
            a=SCRAPER_LINEAR_A,
        )
        movel(
            posx(-150, 0, 0, 0, 0, 0),
            mod=DR_MV_MOD_REL,
            ref=DR_BASE,
            v=SCRAPER_LINEAR_V,
            a=SCRAPER_LINEAR_A,
        )
        movel(
            posx(0, -250, 0, 0, 0, 0),
            mod=DR_MV_MOD_REL,
            ref=DR_BASE,
            v=SCRAPER_LINEAR_V,
            a=SCRAPER_LINEAR_A,
        )
        movej(posj(*DISPENSING_POINT), vel=SCRAPER_VELOCITY, acc=SCRAPER_ACC)
        movel(
            posx(-100, 0, 0, 0, 0, 0),
            mod=DR_MV_MOD_REL,
            ref=DR_BASE,
            v=SCRAPER_LINEAR_V,
            a=SCRAPER_LINEAR_A,
        )

    # ------------------------------------------------------------
    # 3단계: 스크래퍼에 담긴 약을 봉투에 붓고 스크래퍼 반납
    # ------------------------------------------------------------
    def pour_and_return_scraper(self):
        self.node.get_logger().info("[POUR 1/3] 봉투 위치 위로 이동")
        movej(posj(*POUCH_POS_ABOVE), vel=SCRAPER_VELOCITY, acc=SCRAPER_ACC)

        self.node.get_logger().info("[POUR 2/3] 봉투에 약 붓기")
        movej(posj(*POUCH_POS), vel=SCRAPER_VELOCITY, acc=SCRAPER_ACC)
        sleep(0.5)

        self.node.get_logger().info("[POUR 3/3] 스크래퍼 거치대로 복귀 및 반납")
        movej(posj(*SCRAPER_RETURN_MIDDLE), vel=SCRAPER_VELOCITY, acc=SCRAPER_ACC)
        sleep(0.5)
        movej(posj(*SCRAPER_RETURN_STAND), vel=SCRAPER_VELOCITY, acc=SCRAPER_ACC)

        self.release()
        movel(
            posx(-20, 0, 0, 0, 0, 0),
            mod=DR_MV_MOD_REL,
            ref=DR_BASE,
            v=SCRAPER_LINEAR_V,
            a=SCRAPER_LINEAR_A,
        )
        self.grip()

        set_tcp(TCP_GRIPPER_ONLY)
        self.node.get_logger().info(f"TCP 전환: {TCP_GRIPPER_ONLY}")

        movej(posj(*HOME), vel=SCRAPER_VELOCITY, acc=SCRAPER_ACC)

    # ------------------------------------------------------------
    # 전체 테스트 동작
    # ------------------------------------------------------------
    def run_full_sequence(self) -> bool:
        self.pickup_scraper_and_wait()
        self.pour_and_return_scraper()

        if not self.run_paper_bag_sequence():
            self.node.get_logger().warn("종이봉투 작업 실패로 전체 테스트를 중단합니다.")
            return False

        self.publish_task_done()
        return True

    def publish_task_done(self):
        done_msg = Bool()
        done_msg.data = True
        self.task_done_pub.publish(done_msg)
        self.node.get_logger().info("manipulator_test_2 task_done published: True")
        rclpy.spin_once(self.node, timeout_sec=0.1)


def main(args=None):
    global movej, movel, set_tool, set_tcp
    global set_digital_output, get_current_posj
    global DR_BASE, DR_MV_MOD_REL
    global posj, posx
    global wait

    rclpy.init(args=args)
    node = rclpy.create_node("manipulator_test_2", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            movej,
            movel,
            set_tool,
            set_tcp,
            set_digital_output,
            get_current_posj,
            DR_BASE,
            DR_MV_MOD_REL,
            wait,
        )
        from DR_common2 import posj, posx
    except ImportError as e:
        node.get_logger().error(f"Error importing DSR_ROBOT2: {e}")
        node.destroy_node()
        rclpy.shutdown()
        return

    try:
        set_tool("Tool Weight_1")
        set_tcp(TCP_GRIPPER_ONLY)

        manipulator = ManipulatorTest2(node)

        if "--paper-only" in sys.argv:
            if manipulator.run_paper_bag_sequence():
                manipulator.publish_task_done()
        elif "--scraper-only" in sys.argv:
            manipulator.pickup_scraper_and_wait()
            manipulator.pour_and_return_scraper()
            manipulator.publish_task_done()
        else:
            manipulator.run_full_sequence()

    except Exception as e:
        node.get_logger().error(f"manipulator_test_2 실행 중 오류: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

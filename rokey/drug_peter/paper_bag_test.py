# paper_bag_test.py 
# - 종이 봉투 위치로 가기
# - 종이 봉투 gripping
# - 종이 봉투 살짝 아래로 내리면서 빼기
# - 수납대로 이동
# - 수납대 위로 내려놓기 및 작업 완료 상태 pub

#-------------------------------------------ver2---------------------------------------------------------------------------

import rclpy
import DR_init 
from time import sleep
from std_msgs.msg import Bool

ROBOT_ID   = "dsr01"
ROBOT_MODEL= "m0609"
VELOCITY, ACC = 30, 30 # 속도 조절

DR_init.__dsr__id   = ROBOT_ID
DR_init.__dsr__model= ROBOT_MODEL

ON, OFF = 1, 0
CYCLE = 3
CHECK_INTERVAL = 0.1

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("paper_bag_test", namespace=ROBOT_ID)
    paper_bag_done_pub = node.create_publisher(
    Bool,
    "task_done",
    10
    )

    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            set_digital_output,
            get_digital_input,
            set_tool,
            set_tcp,
            movej,
			movel,
			task_compliance_ctrl,
			get_tool_force,
			amove_periodic,
			check_position_condition,
            wait,
            trans,
            set_desired_force,
            DR_BASE,
            DR_MV_MOD_ABS,
            DR_MV_MOD_REL,
            DR_AXIS_Z,
            DR_SSTOP,
            get_current_posx,
            get_current_posj,
            release_force,
        )

        from DR_common2 import posj, posx

    except ImportError as e:
        node.get_logger().info(f"Error importing DSR_ROBOT2 : {e}")
        return
	
    set_tool("Tool Weight_1") # 패드에 있는 이름으로 해야 함 
    set_tcp('Tool_v1') # 패드에 있는 이름으로 해야 함
# ---------------------------------------------grip 함수-----------------------------------------------------------------------

    def ungrip(): # 너비 80MM, 40N
        node.get_logger().info("set for digital output 1 0 for ungrip")
        set_digital_output(1, OFF)
        set_digital_output(2, OFF)
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        sleep(1)

    def grip(): # 너비 2MM, 20N 
        node.get_logger().info("set for digital output 0 1 for grip")
        set_digital_output(1, OFF)
        set_digital_output(2, OFF)
        set_digital_output(1, OFF)
        set_digital_output(2, ON)
        sleep(1)    
# ---------------------------------------------작업 시작-----------------------------------------------------------------------
    #go to home
    node.get_logger().info("paper_bag_test 실행.")
    ungrip()
    node.get_logger().info("집으로 출발.")
    movej(posj(0, 0, 90, 0, 90, 0), vel=VELOCITY, acc=ACC) # go to home
    wait(0.5)

    # 봉투 위로 가기
    node.get_logger().info("봉투 위로 가는 중....")
    # movej(posj(-40.6, 42.90, 31.34, -0.07, 105.34, -39.66), vel=VELOCITY, acc=ACC) # ver1 위치 
    movej(posj(-36.32, 53.19, 28.79, -0.59, 98.47, -130.76), vel=VELOCITY, acc=ACC) # ver2 위치
    node.get_logger().info("봉투 위에 도착")
    wait(0.5)

    # 봉투로 내려가서 잡기
    node.get_logger().info("봉투로 내려가는 중...")
    # movej(posj(-39.95, 49.59, 66.92, -0.07, 63.44, -39.65), vel=VELOCITY, acc=ACC) # ver1 위치 
    movej(posj(-37.34, 57.57, 46.94, -0.61, 75.93, -131.56), vel=VELOCITY, acc=ACC) # ver2 위치
    grip()
    node.get_logger().info("봉투 잡음!!")
    
    # 봉투를 아래로 살짝 빼기: base 기준 Z -30mm
    wait(0.5)
    node.get_logger().info("봉투 아래로 살짝 빼는 중...")
    movel(
        posx(0, 0, -100, 0, 0, 0),
        mod=DR_MV_MOD_REL,
        ref=DR_BASE,
        v=100,
        a=50
    )
    node.get_logger().info("봉투 빼기 완료.")

    # 위로 올라가기
    wait(0.5)
    node.get_logger().info("위로 올라가는 중...")
    movel(
        posx(0, -50, 300, 0, 0, 0),
        mod=DR_MV_MOD_REL,
        ref=DR_BASE,
        v=50,
        a=50
    )
    node.get_logger().info("위로 올라감.")

    
    # -y 방향으로 좀 빼기
    wait(0.5)
    node.get_logger().info("-y 방향으로 좀 빼는중...")
    movej(posj(-79.23, 48.44, 36.97, -0.07, 94.56, -42.92), vel=VELOCITY, acc=ACC)
    node.get_logger().info("동작완료")

    # 가상 수납대 위로 가기 
    wait(0.5)
    node.get_logger().info("수납대 위로 가는 중...")
    movej(posj(-51.77, -8.32, 84.92, -0.72, 101.25, -52.54), vel=VELOCITY, acc=ACC)

    # 가상 수납대 위로 올라갔으면 ungrip 하기!
    current_posj = get_current_posj()
    if (-52.00 <= current_posj[0] <= -50.00) and (-9.00 <= current_posj[1] <= -8.00) and (84.00 <= current_posj[2] <= 86.00):
    # 목표 3차원 joint 구역에 도달했을 때 실행할 코드
        ungrip()
        node.get_logger().info("목표 지점 도달 완료!")

#       ------------------------작업 완료 pub 코드--------------------------------------
        done_msg = Bool()
        done_msg.data = True
        paper_bag_done_pub.publish(done_msg)
        node.get_logger().info("paper bag task done published: True")

        rclpy.spin_once(node, timeout_sec=0.1)
        wait(0.5)
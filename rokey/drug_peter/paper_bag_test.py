# paper_bag_test.py
# - 종이 봉투 위치로 가기
# - 종이 봉투 gripping
# - 종이 봉투 살짝 아래로 내리면서 빼기
# - 수납대로 이동
# - 수납대 위로 내려놓기 및 작업 완료 상태 pub

import rclpy
import DR_init 
from time import sleep
# from DR_common2 import *
# from dsr_msgs2.srv import MoveStop

ROBOT_ID   = "dsr01"
ROBOT_MODEL= "m0609"
VELOCITY, ACC = 30, 30

DR_init.__dsr__id   = ROBOT_ID
DR_init.__dsr__model= ROBOT_MODEL

ON, OFF = 1, 0
CYCLE = 3
CHECK_INTERVAL = 0.1

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("paper_bag_test", namespace=ROBOT_ID)

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
            release_force,
        )

        from DR_common2 import posj, posx

    except ImportError as e:
        node.get_logger().info(f"Error importing DSR_ROBOT2 : {e}")
        return
	
    set_tool("Tool Weight_1") # 패드에 있는 이름으로 해야 함 
    set_tcp('Tool_v1') # 패드에 있는 이름으로 해야 함

    def ungrip():
        node.get_logger().info("set for digital output 1 0 for ungrip")
        set_digital_output(1, OFF)
        set_digital_output(2, OFF)
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        sleep(1)

    def grip():
        node.get_logger().info("set for digital output 0 1 for grip")
        set_digital_output(1, OFF)
        set_digital_output(2, OFF)
        set_digital_output(1, OFF)
        set_digital_output(2, ON)
        sleep(1)    
# --------------------------------------------------------------------------------------------------------------------
    #go to home
    ungrip()
    movej(posj(0, 0, 90, 0, 90, 0), vel=VELOCITY, acc=ACC) #go to home
    wait(0.5)

    # 봉투 위로 가기
    movej(posj(-40.6, 42.90, 31.34, -0.07, 105.34, -39.66), vel=VELOCITY, acc=ACC)
    wait(0.5)

    # 봉투로 내려가서 잡기
    movej(posj(-39.95, 49.59, 66.92, -0.07, 63.44, -39.65), vel=VELOCITY, acc=ACC)
    grip()
    
    # 봉투를 아래로 살짝 빼기: base 기준 Z -30mm
    wait(0.5)
    movel(
        posx(0, 0, -100, 0, 0, 0),
        mod=DR_MV_MOD_REL,
        ref=DR_BASE,
        v=100,
        a=50
    )

    # 위로 올라가기
    wait(0.5)
    movel(
        posx(0, -50, 300, 0, 0, 0),
        mod=DR_MV_MOD_REL,
        ref=DR_BASE,
        v=50,
        a=50
    )

    

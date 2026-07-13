# go_home.py
# 0.0.90.0.90.0 으로 가는 코드


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
            set_tool,
            set_tcp,
            movej,

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
# --------------------------------------------------------------------------------------------------------------------
    #go to home
    ungrip()
    movej(posj(0, 0, 90, 0, 90, 0), vel=VELOCITY, acc=ACC) #go to home
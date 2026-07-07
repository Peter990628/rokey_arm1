# pick and place in 1 method. from pos1 to pos2 @20241104
# ros2 launch m0609_rg2_bringup bringup.launch.py mode:=real host:=192.168.1.100 port:=12345 model:=m0609
import rclpy
import DR_init 
from time import sleep

ROBOT_ID   = "dsr01"
ROBOT_MODEL= "m0609"
VELOCITY, ACC = 30, 30

DR_init.__dsr__id   = ROBOT_ID
DR_init.__dsr__model= ROBOT_MODEL

ON, OFF = 1, 0

class PourPills(Node):
    def __init__(self, node):
        self.node = node

        from DSR_ROBOT2 import (
            set_digital_output,
            get_digital_input,
            set_tool,
            set_tcp,
            movej,
            movel,
            wait,
            trans,
            task_compliance_ctrl,
            get_tool_force,
            amove_periodic,
            check_position_condition,
            set_desired_force,
            get_current_posx,
            release_force,
            DR_BASE,
            DR_MV_MOD_ABS,
            DR_MV_MOD_REL,
            DR_AXIS_Z,
            DR_SSTOP

        )

        from DR_common2 import posj, posx

        self.set_digital_output = set_digital_output
        self.get_digital_input = get_digital_input
        self.set_tool = set_tool
        self.set_tcp = set_tcp
        self.movej




def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("gear_test", namespace=ROBOT_ID)

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

    def release():
        node.get_logger().info("set for digital output 1 0 for grip")
        set_digital_output(1, OFF)
        set_digital_output(2, OFF)
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        sleep(1)

    def grip():
        node.get_logger().info("set for digital output 0 1 for release")
        set_digital_output(1, OFF)
        set_digital_output(2, OFF)
        set_digital_output(1, OFF)
        set_digital_output(2, ON)
        sleep(1)

    def grasp():
        # TODO : 집게 잡기
    
    def pour():
        # TODO : 집게 잡고 붓기
    
    def move():
        # TODO : 조제기로 이동 / 쓰레기 이동
    
    def close():
        # TODO : 서랍 닫기 
    


# pick and place in 1 method. from pos1 to pos2 @20241104
# ros2 launch m0609_rg2_bringup bringup.launch.py mode:=real host:=192.168.1.100 port:=12345 model:=m0609
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

    # move_stop_client = node.create_client(MoveStop, "motion/move_stop")

    # def stop_motion(stop_mode=DR_SSTOP):
    #     if not move_stop_client.wait_for_service(timeout_sec=1.0):
    #         node.get_logger().error("motion/move_stop service is not available")
    #         return False

    #     req = MoveStop.Request()
    #     req.stop_mode = stop_mode

    #     future = move_stop_client.call_async(req)
    #     rclpy.spin_until_future_complete(node, future)

    #     try:
    #         result = future.result()
    #     except Exception as e:
    #         node.get_logger().error(f"motion/move_stop service call failed: {e}")
    #         return False

    #     if result is None or not result.success:
    #         node.get_logger().error("motion/move_stop failed")
    #         return False

    #     node.get_logger().info("motion/move_stop success")
    #     return True
	
    set_tool("Tool Weight_1") # 패드에 있는 이름으로 해야 함 
    set_tcp("GripperDA_v1") # 패드에 있는 이름으로 해야 함

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


    set_tcp('Tool_v1')
    delta = [0,0,-80,0,0,0]
    delta_2 = [0,0,80,0,0,0]



    movej(posj(0, 0, 90, 0, 90, 0), vel=VELOCITY, acc=ACC) #go to home
    task_compliance_ctrl([10000,10000,200,10000,10000,10000])



    # set_desired_force([0,0,-70,0,0,0],[0,0,1,0,0,0])
    force_ext = get_tool_force(DR_BASE)

    release()
    # -------1 시작 --------
    x1 = posx(365.20, 145.88, 118.47, 83.66, -179.60, 77.02)
    movel(posx(365.20, 145.88, 118.47, 83.66, -179.60, 77.02), mod=DR_MV_MOD_ABS, ref=DR_BASE , radius = 70, v = 1500, a = 1500)
    movel(posx(0,0,-80,0,0,0), mod = DR_MV_MOD_REL, radius = 70, v = 1500, a = 1500)
    # move_delta = trans(x1 , delta, DR_BASE,DR_BASE)
    # movel(move_delta, mod=DR_MV_MOD_ABS, ref=DR_BASE , radius = 70, v = 1500, a = 1500)
    # move_delta_2 = trans(x1 , delta_2, DR_BASE, DR_BASE)
    grip()
    movel(posx(0,0,80,0,0,0), mod = DR_MV_MOD_REL, radius = 70, v = 1500, a = 1500)
    # movel(move_delta_2 ,mod=DR_MV_MOD_ABS, ref=DR_BASE , radius = 70, v = 1500, a = 1500)
    x2 = posx(603.45, 53.06, 128.42, 131.71, -179.99, 110.31)
    # move_delta_3 = trans(x2, delta, DR_BASE,DR_BASE)
    # movel(move_delta_3,mod=DR_MV_MOD_ABS, ref=DR_BASE , radius = 70, v = 1500, a = 1500)
    movel(posx(603.45, 53.06, 128.42, 131.71, -179.99, 110.31), mod=DR_MV_MOD_ABS, ref=DR_BASE  , radius = 70, v = 1500, a = 1500)
    movel(posx(603.45, 53.06, 48.42, 131.71, -179.99, 110.31), mod=DR_MV_MOD_ABS, ref=DR_BASE  , radius = 70, v = 1500, a = 1500)
    release()
    movel(posx(0,0,60,0,0,0), mod = DR_MV_MOD_REL, radius = 70, v = 1500, a = 1500)
    # -------1 끝 --------


    # -------2 시작--------

    movel(posx(456.18, 207.35, 113.94, 104.50, -179.58, 97.99), mod=DR_MV_MOD_ABS, ref=DR_BASE  ,radius = 70, v = 1500, a = 1500)
    movel(posx(0,0,-80,0,0,0), mod = DR_MV_MOD_REL, radius = 70, v = 1500, a = 1500)
    grip()
    movel(posx(0,0,80,0,0,0), mod = DR_MV_MOD_REL, radius = 70, v = 1500, a = 1500)
    movel(posx(513.30, -5.58, 138.30, 102.11, -179.58, 101.50), mod=DR_MV_MOD_ABS, ref=DR_BASE  , radius = 70, v = 1500, a = 1500)
    movel(posx(513.30, -5.58, 58.30, 102.11, -179.58, 101.50), mod=DR_MV_MOD_ABS, ref=DR_BASE ,radius = 70, v = 1500, a = 1500)
    release()
    movel(posx(0,0,60,0,0,0), mod = DR_MV_MOD_REL, radius = 70, v = 1500, a = 1500)

    # -------2 끝 --------


    # -------3 시작 --------

    movel(posx(460.20, 101.83, 112.96, 72.33, -179.41, 65.80), mod=DR_MV_MOD_ABS, ref=DR_BASE ,radius = 70, v = 1500, a = 1500)
    movel(posx(0,0,-80,0,0,0), mod = DR_MV_MOD_REL, radius = 70, v = 1500, a = 1500)
    grip()
    movel(posx(0,0,80,0,0,0), mod = DR_MV_MOD_REL, radius = 70, v = 1500, a = 1500)
    movel(posx(607.99, -51.66, 145.46, 106.35, -178.97, 106.85), mod=DR_MV_MOD_ABS, ref=DR_BASE  , radius = 70, v = 1500, a = 1500)
    movel(posx(607.99, -51.66, 65.46, 106.35, -178.97, 106.85), mod=DR_MV_MOD_ABS, ref=DR_BASE , radius = 70, v = 1500, a = 1500)
    release()
    movel(posx(0,0,60,0,0,0), mod = DR_MV_MOD_REL, radius = 70, v = 1500, a = 1500)


    # -------3 끝 --------


    # -------4 시작 --------
    movel(posx(427.19, 149.73, 112.20, 55.08, -179.07, 48.14), mod=DR_MV_MOD_ABS, ref=DR_BASE , radius = 70, v = 1500, a = 1500)
    movel(posx(0,0,-80,0,0,0), mod = DR_MV_MOD_REL, radius = 70, v = 1500, a = 1500)
    grip()
    movel(posx(0,0,80,0,0,0), mod = DR_MV_MOD_REL, radius = 70, v = 1500, a = 1500)
    movel(posx(574.65, -0.18, 144.17,20.60,-178.99, 15.24), mod=DR_MV_MOD_ABS, ref=DR_BASE  , radius = 70, v = 1500, a = 1500)
    movel(posx(574.65, -0.18, 64.17,20.60,-178.99, 15.24), mod=DR_MV_MOD_ABS, ref=DR_BASE  , radius = 70, v = 1500, a = 1500)
    node.get_logger().info("move done")
    wait(0.5)
    set_desired_force([0,0,-10,0,0,0],[0,0,1,0,0,0])
    wait(0.5)
    node.get_logger().info("set_desired_force done")
    wait(0.5)
    amove_periodic(amp =[0,0,0,0,0,8], period=[0,0,0,0,0,1.5], atime=0.5, repeat=5, ref=DR_BASE)
    wait(0.5)
    node.get_logger().info("amove_periodic_start")  
    wait(0.5)
    # while True:
    #     # node.get_logger().info("amove_periodic_start")
    #     # amove_periodic(amp =[0,0,0,0,0,8], period=[0,0,0,0,0,1.5], atime=0.5, repeat=10, ref=DR_BASE)
    #     node.get_logger().info("amove_periodic_done")
    #     wait(0.5)
    #     if check_position_condition(DR_AXIS_Z,43,46,DR_BASE,DR_MV_MOD_ABS):
    #         # stop_motion(DR_SSTOP)
    #         wait(0.5)
    #         node.get_logger().info("check_position_condition_done")
    #         wait(0.5)
    #         break

    wait(0.5)
    set_desired_force([0,0,-20,0,0,0],[0,0,1,0,0,0])
    while True:
        if check_position_condition(DR_AXIS_Z,0,38,DR_BASE,DR_MV_MOD_ABS) == 0 :
            current_posx, _ = get_current_posx(DR_BASE)
            node.get_logger().info(f"current z = {current_posx[2]}")
            
            release()
            wait(0.5)
            release_force()
            wait(0.5)
            movel(posx(0,0,60,0,0,0), mod = DR_MV_MOD_REL, v = 1500, a = 1500)
            wait(0.5)
            movej(posj(0, 0, 90, 0, 90, 0), vel=VELOCITY, acc=ACC)
            break
    # -------4 끝 --------

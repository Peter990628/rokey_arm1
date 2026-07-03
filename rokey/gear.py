# pick and place in 1 method. from pos1 to pos2 @20241104
import rclpy
import DR_init 

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
            
        )

        from DR_common2 import posj, posx 

    except ImportError as e:
        node.get_logger().info(f"Error importing DSR_ROBOT2 : {e}")
        return
	
    set_tool("Tool Weight_1") # 패드에 있는 이름으로 해야 함 
    set_tcp("GripperDA_v1") # 패드에 있는 이름으로 해야 함

    def grip():
        set_digital_output([-1, -2])
        set_digital_output(2)
        wait(0.5)

    def ungrip():
        set_digital_output([-1,-2])
        set_digital_output(1)
        wait(0.3)


    set_tcp('Tool_v1')
    delta = [0,0,-80,0,0,0]
    delta_2 = [0,0,80,0,0,0]



    movej(posj(0,0,90,0,90,0))
    task_compliance_ctrl([10000,10000,200,10000,10000,10000])



    # set_desired_force([0,0,-70,0,0,0],[0,0,1,0,0,0])
    force_ext = get_tool_force(DR_BASE)

    ungrip()
    # -------1 시작 --------
    x1 = posx(365.20, 145.88, 118.47, 83.66, -179.60, 77.02)
    # movel(posx(365.20, 145.88, 118.47, 83.66, -179.60, 77.02), mod=DR_MV_MOD_ABS, ref=DR_BASE , radius = 70, v = 1500, a = 1500)
    # movel(posx(0,0,-80,0,0,0), mod = DR_MV_MOD_REL, radius = 70)
    move_delta = trans(x1 , delta, DR_BASE,DR_BASE)
    movel(move_delta, mod=DR_MV_MOD_ABS, ref=DR_BASE , radius = 70, v = 1500, a = 1500)
    move_delta_2 = trans(x1 , delta_2, DR_BASE, DR_BASE)
    grip()
    # movel(posx(0,0,80,0,0,0), mod = DR_MV_MOD_REL, radius = 70, v = 1500, a = 1500)
    movel(move_delta_2 ,mod=DR_MV_MOD_ABS, ref=DR_BASE , radius = 70, v = 1500, a = 1500)
    x2 = posx(603.45, 53.06, 128.42, 131.71, -179.99, 110.31)
    move_delta_3 = trans(x2, delta, DR_BASE,DR_BASE)
    movel(move_delta_3,mod=DR_MV_MOD_ABS, ref=DR_BASE , radius = 70, v = 1500, a = 1500)
    # movel(posx(603.45, 53.06, 128.42, 131.71, -179.99, 110.31), mod=DR_MV_MOD_ABS, ref=DR_BASE  , radius = 70, v = 1500, a = 1500)
    # movel(posx(603.45, 53.06, 48.42, 131.71, -179.99, 110.31), mod=DR_MV_MOD_ABS, ref=DR_BASE  , radius = 70, v = 1500, a = 1500)
    ungrip()
    movel(posx(0,0,60,0,0,0), mod = DR_MV_MOD_REL, radius = 70, v = 1500, a = 1500)
    # -------1 끝 --------


    # -------2 시작--------

    movel(posx(456.18, 207.35, 113.94, 104.50, -179.58, 97.99), mod=DR_MV_MOD_ABS, ref=DR_BASE  ,radius = 70, v = 1500, a = 1500)
    movel(posx(0,0,-80,0,0,0), mod = DR_MV_MOD_REL, radius = 70)
    grip()
    movel(posx(0,0,80,0,0,0), mod = DR_MV_MOD_REL, radius = 70, v = 1500, a = 1500)
    movel(posx(513.99, -5.83, 127.46, 46.11, 180.00, 36.58), mod=DR_MV_MOD_ABS, ref=DR_BASE  , radius = 70, v = 1500, a = 1500)
    movel(posx(513.99, -5.83, 47.46, 46.11, 180.00, 36.58), mod=DR_MV_MOD_ABS, ref=DR_BASE ,radius = 70, v = 1500, a = 1500)
    ungrip()
    movel(posx(0,0,60,0,0,0), mod = DR_MV_MOD_REL, radius = 70, v = 1500, a = 1500)

    # -------2 끝 --------


    # -------3 시작 --------

    movel(posx(460.20, 101.83, 112.96, 72.33, -179.41, 65.80), mod=DR_MV_MOD_ABS, ref=DR_BASE ,radius = 70, v = 1500, a = 1500)
    movel(posx(0,0,-80,0,0,0), mod = DR_MV_MOD_REL, radius = 70)
    grip()
    movel(posx(0,0,80,0,0,0), mod = DR_MV_MOD_REL, radius = 70, v = 1500, a = 1500)
    movel(posx(608.13, -52.07, 125.64, 175.64, -179.87, 166.24), mod=DR_MV_MOD_ABS, ref=DR_BASE  , radius = 70, v = 1500, a = 1500)
    movel(posx(608.13, -52.07, 45.64, 175.64, -179.87, 166.24), mod=DR_MV_MOD_ABS, ref=DR_BASE , radius = 70, v = 1500, a = 1500)
    ungrip()
    movel(posx(0,0,60,0,0,0), mod = DR_MV_MOD_REL, radius = 70, v = 1500, a = 1500)


    # -------3 끝 --------


    # -------4 시작 --------
    movel(posx(427.19, 149.73, 112.20, 55.08, -179.07, 48.14), mod=DR_MV_MOD_ABS, ref=DR_BASE , radius = 70, v = 1500, a = 1500)
    movel(posx(0,0,-80,0,0,0), mod = DR_MV_MOD_REL, radius = 70)
    grip()
    movel(posx(0,0,80,0,0,0), mod = DR_MV_MOD_REL, radius = 70, v = 1500, a = 1500)
    movel(posx(576.50, -2.30, 105.52, 54.68, 179.90, 47.50), mod=DR_MV_MOD_ABS, ref=DR_BASE  , radius = 70, v = 1500, a = 1500)
    movel(posx(576.50, -2.30, 47, 54.68, 179.90, 47.50), mod=DR_MV_MOD_ABS, ref=DR_BASE  , radius = 70, v = 1500, a = 1500)
    set_desired_force([0,0,-10,0,0,0],[0,0,1,0,0,0])

    amove_periodic(amp =[0,0,0,0,0,8], period=[0,0,0,0,0,1.5], atime=0.5, repeat=10, ref=DR_BASE)
    while True:
        if check_position_condition(DR_AXIS_Z,43,46,DR_BASE,DR_MV_MOD_ABS):
            stop(DR_SSTOP)
            break

    set_desired_force([0,0,-10,0,0,0],[0,0,1,0,0,0])
    while True:
        if check_position_condition(DR_AXIS_Z,0,38,DR_BASE,DR_MV_MOD_ABS):
            ungrip()
            movel(posx(0,0,60,0,0,0), mod = DR_MV_MOD_REL )
            break
    # -------4 끝 --------
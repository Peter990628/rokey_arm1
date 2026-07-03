# grip_test.py
# /mnt/697d97b3-9148-4fdb-921c-4a1785efc108/ws_cobot_pjt/ws_dsr/install/dsr_common2/lib/dsr_common2/imp
# pick and place in 1 method. from pos1 to pos2 @20241104
import rclpy
import DR_init
from time import sleep

# for single robot
ROBOT_ID   = "dsr01"
ROBOT_MODEL= "m0609"
VELOCITY, ACC = 30, 30

DR_init.__dsr__id   = ROBOT_ID
DR_init.__dsr__model= ROBOT_MODEL
# ------------------------------------------------------------------------------------
ON, OFF = 1, 0
CYCLE = 3
CHECK_INTERVAL = 0.1


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("rokey_grip_simple", namespace=ROBOT_ID) #함수로 만드는 노드 -> 간단하게 만들 때 사용
    # 이 노드 뒤에 아래꺼 작성해야 함!!!!
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            set_digital_output,
            get_digital_input,
            set_tool,
            set_tcp,
            movej,
            wait,
        )

        from DR_common2 import posj

    except ImportError as e:
        node.get_logger().info(f"Error importing DSR_ROBOT2 : {e}")
        return

    set_tool("Tool Weight_1") # 패드에 있는 이름으로 해야 함 
    set_tcp("GripperDA_v1") # 패드에 있는 이름으로 해야 함
# --------------------------------------------여기까지는 거의 안 바뀜 

    def grip():
        node.get_logger().info("set for digital output 1 0 for grip")
        set_digital_output(1, OFF)
        set_digital_output(2, OFF)
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        sleep(1)

    def release():
        node.get_logger().info("set for digital output 0 1 for release")
        set_digital_output(1, OFF)
        set_digital_output(2, OFF)
        set_digital_output(1, OFF)
        set_digital_output(2, ON)
        sleep(1)
     
    homej = posj([0, 0, 90, 0, 90, 0])

    node.get_logger().info(f"Moving to joint position: {homej}")
    movej(homej, vel=VELOCITY, acc=ACC)

    try:
        for i in range(CYCLE):
            node.get_logger().info(f"=== Cycle {i+1}/{CYCLE} ===")

            grip()
            release()
            sleep(1)

        node.get_logger().info("Gripper Test Complete")

    except KeyboardInterrupt:
        node.get_logger().info("Program Stopped")
    except TimeoutError as e:
        node.get_logger().error(str(e))

    finally:
        try:
            movej(homej, vel=VELOCITY, acc=ACC)
        except Exception as e:
            node.get_logger().error(f"Failed to move home: {e}")

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


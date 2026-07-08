import rclpy
import DR_init
from time import sleep
from rclpy.node import Node
import requests
import json
from std_msgs.msg import String

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 30, 30

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

ON, OFF = 1, 0


class PourPills:
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
            task_compliance_ctrl,
            set_desired_force,
            get_current_posx,
            release_force,
            DR_BASE,
            DR_MV_MOD_ABS,
            DR_MV_MOD_REL,
            DR_AXIS_Z,
            DR_SSTOP,
        )

        from DR_common2 import posj, posx

        # DSR 함수 저장
        self.set_digital_output = set_digital_output
        self.get_digital_input = get_digital_input
        self.set_tool = set_tool
        self.set_tcp = set_tcp
        self.movej = movej
        self.movel = movel
        self.wait = wait
        self.task_compliance_ctrl = task_compliance_ctrl
        self.set_desired_force = set_desired_force
        self.get_current_posx = get_current_posx
        self.release_force = release_force

        # DSR 상수 저장
        self.DR_BASE = DR_BASE
        self.DR_MV_MOD_ABS = DR_MV_MOD_ABS
        self.DR_MV_MOD_REL = DR_MV_MOD_REL
        self.DR_AXIS_Z = DR_AXIS_Z
        self.DR_SSTOP = DR_SSTOP

        # 위치 생성 함수
        self.posj = posj
        self.posx = posx

        # 기본 속도 / 가속도
        self.vel = VELOCITY
        self.acc = ACC

        # 자주 쓰는 위치 정의
        self.define_positions()
        
        # task_manager에서 나눠주는 좌표 
        self.create_subscriptions()

        # 로봇 초기 세팅
        self.init_robot()

    # --------------------------------------------------
    # 0. 위치 정의
    # --------------------------------------------------
    def define_positions(self):
        
        self.X_TRASH_DROP = posx()      # 쓰레기통 위치
        self.J_TWEEZER_HOME = posj()    # 집게 툴 거치대 위치
        self.X_TWEEZER_NEAR = posx()    # 집게 툴 거치대 주변 위치     
    # ----------------------------
    # 구독하는거
    # ----------------------------------
    def create_subscriptions(self):
        self.sub_medicine = self.create_subscription(String, 'dsr01/pharmacy/medicine', self.dispenser_pose_callback, 10)
        self.get_logger().info("구독함")

    def dispenser_pose_callback(self, msg):
        try:
            data = json.loads(msg.data)

            task = data[0]
            self.set_task_from_data(task)

        except Exception as e:
            self.get_logger().error(f"작업데이터받기실패:{e}")

    def set_task_from_data(self, task):
        required_keys = [
        "id",
        "medicine_name",
        "dispensing_x",
        "dispensing_y",
        "dispensing_z",
        "lid_type"
    ]
        for key in required_keys:
            if key not in task:
                raise KeyError(f"{key}가 없음")
        
        self.current_task = task
        self.medicine_id = task["id"]
        self.medicine_name = task["medicine_name"]
        self.dispensing_x = float(task["dispensing_x"])
        self.dispensing_y = float(task["dispensing_y"])
        self.dispensing_z = float(task["dispensing_z"])
        self.lid_type = task["lid_type"]

        self.task_ready = False

        self.X_DISPENSER_POS = self.posx(
            self.dispensing_x,
            self.dispensing_y,
            self.dispensing_z,
            rx,
            ry,
            rz # 이것들은 재보고 
        )

        self.X_CLOSE_DRAWER = self.posx(
            self.dispensing_x + dx, #dx값 해보고
            self.dispensing_y + dy,
            self.dispensing_z,
            rx,
            ry,
            rz
        )
    # --------------------------------------------------
    # 1. 로봇 초기 세팅
    # --------------------------------------------------
    def init_robot(self):
        self.node.get_logger().info("로봇 초기 세팅 시작")

        # 실제 등록된 tool/tcp 이름으로 수정 필요
        self.set_tool("Tool Weight_2FG")
        self.set_tcp("2FG_TCP")

        self.release()

        self.node.get_logger().info("약 붓기 시작")

    # --------------------------------------------------
    # 2. 그리퍼 열기
    # --------------------------------------------------
    def release(self):
        self.node.get_logger().info("그리퍼 열기")

        self.set_digital_output(1, OFF)
        self.set_digital_output(2, OFF)

        self.set_digital_output(1, ON)
        self.set_digital_output(2, OFF)

        sleep(1)

    # --------------------------------------------------
    # 3. 그리퍼 닫기
    # --------------------------------------------------
    def grip(self):
        self.node.get_logger().info("그리퍼 닫기")

        self.set_digital_output(1, OFF)
        self.set_digital_output(2, OFF)

        self.set_digital_output(1, OFF)
        self.set_digital_output(2, ON)

        sleep(1)
    
    def little_release(self):
        self.node.get_logger().info("그리퍼 닫기")

        self.set_digital_output(1, OFF)
        self.set_digital_output(2, OFF)

        self.set_digital_output(1, OFF)
        self.set_digital_output(2, ON)

        sleep(1)

    # --------------------------------------------------
    # 5. 조제기 / 서랍 쪽으로 이동
    # --------------------------------------------------
    def move_pour_tweezer(self):

        self.node.get_logger().info("조제기 위치로 이동 시작")

        self.movejx(
            self.X_DISPENSER_POS,
            vel=self.vel,
            acc=self.acc,
            ref=self.DR_BASE,
            sol=2
        )

        # 필요 시 force control 적용
        self.contact_dispenser()

        # 약 붓기
        self.pour_tweezer()

        self.node.get_logger().info("조제기 위치 작업 완료")

    # --------------------------------------------------
    # 6. 조제기 밀착 force control
    # --------------------------------------------------
    def contact_dispenser_tweezer(self):
        self.node.get_logger().info("조제기 밀착 제어 시작")

        self.set_tcp("집게 끝")
        cur_pos1 = self.get_current_posx()[0]
        x = cur_pos1[0]
        y = cur_pos1[1]
        z = cur_pos1[2]
        rx = cur_pos1[3]
        ry = cur_pos1[4]
        rz = cur_pos1[5]

        contact_pos = posx(x, y, z, rx1, rx2, rx3)
        # 얼마나 + - 해줄지는 한번 해보고 결정 
        movel(contact_pos, vel, acc)

        try:
            self.task_compliance_ctrl([10000, 500, 10000, 10000, 10000, 10000])

            # 서랍 옆으로 가서 +y? 방향으로 force 주면 될듯 
            self.set_desired_force(
                fd=[0, 0, 0, 0, 0, 0],
                dir=[0, 1, 0, 0, 0, 0],
                mod=self.DR_MV_MOD_REL
            )

            sleep(1)

        finally:
            self.release_force()
            self.node.get_logger().info("조제기 밀착 제어 종료")

    # --------------------------------------------------
    # 7. 약 붓기
    # --------------------------------------------------
    def pour_tweezer(self):
        cur_pos2 = self.get_current_posx()[0]
        x = cur_pos2[0]
        y = cur_pos2[1]
        z = cur_pos2[2]
        rx = cur_pos2[3]
        ry = cur_pos2[4]
        rz = cur_pos2[5]

        pour_start_pose = self.posx(x, y, z, rx, ry, rz)

        pour_tilt_pose = self.posx(
            x,
            y,
            z,
            rx,
            ry,
            rz # 직접 찍어보고 바꾸셈
        )

        # 붓기
        self.movel(pour_tilt_pose, vel=10, acc=10, ref=self.DR_BASE)

        # 다시 돌아오기 
        self.movel(pour_start_pose, vel=10, acc=10, ref=self.DR_BASE)

        # 약 떨어지는 시간
        sleep(2)

        self.node.get_logger().info("약 붓기 완료")

    # --------------------------------------------------
    # 9. 약통 버리러 이동
    # --------------------------------------------------
    def move_trash(self):
        self.node.get_logger().info("약통 버리기 시작")

        self.movel(self.X_TRASH_DROP, vel=10, acc=10, ref=self.DR_BASE)

        self.release() #조금 릴리즈랑 많이 릴리즈 구분할 것 

        self.node.get_logger().info("약통 버리기 완료")

    # --------------------------------------------------
    # 9. 집게 놓기 
    # --------------------------------------------------
    def tool_drop(self):
        self.node.get_logger().info("집게 놓기 시작")

        self.movel(
            self.X_TWEEZER_NEAR, # 근데 뭔가 가서 세워야할듯 
            vel=self.vel,
            acc=self.acc,
            ref=self.DR_BASE
        )

        self.movel(
            self.J_TWEEZER_HOME, # 숫자 해보고 결정
            vel=20,
            acc=20,
            ref=self.DR_BASE,
            mod=self.DR_MV_MOD_REL
        )

        self.release()

        self.node.get_logger().info("집게 놓기 완료")


    # --------------------------------------------------
    # 10. 서랍 닫기
    # --------------------------------------------------
    def close(self):
        self.node.get_logger().info("서랍 닫기 시작")

        self.movel(
            self.X_CLOSE_DRAWER, 
            vel=10,
            acc=10,
            ref=self.DR_BASE
        )

        # 예시: 앞쪽으로 50mm 미는 동작
        # 실제 방향은 좌표계 보고 수정
        self.movel(
            self.posx(-50, 0, 0, 0, 0, 0), # 플러스 마이너스 헷갈림 해보고 결정 
            acc=10,
            ref=self.DR_BASE,
            mod=self.DR_MV_MOD_REL
        )

        # 후퇴
        self.movel(
            self.posx(50, 0, 0, 0, 0, 0),
            vel=10,
            acc=10,
            ref=self.DR_BASE,
            mod=self.DR_MV_MOD_REL
        )

        self.node.get_logger().info("서랍 닫기 완료")

    # --------------------------------------------------
    # 11. DB / task manager 알림
    # --------------------------------------------------
    def notify_done(self):
        self.get_logger().info("리필 완료post 함")
        url = "http://172.23.0.129:8000/api/tasks/refill/"
        payload = {
            "medicine_name": self.medicine_name,
            "amount": self.amount
        }

        try:
            response = requests.post(url, json=payload, timeout=3)
            if response.ok:
                self.get_logger().info(f"리필완 post 보냄: {response.json()}")
            else:
                self.get_logger().error(f"리필포스트보내기실패: status ={response.status_code}, body={response.text}")

        except Exception as e:
            self.get_logger().error(f"예외발생:{e}")        
            
    def run_gripper_lid_task(self):
        """
        일반 집게 방식.
        기존 grasp → move_pour → move_trash 흐름.
        """

        self.node.get_logger().info("집게 방식 작업 시작")

        self.set_tcp(self.TCP_GRIPPER)

        self.grasp()
        self.move_pour()
        self.move_trash()

        self.node.get_logger().info("집게 방식 작업 완료")

    def run_hole_lid_task(self):
        """
        lid_type == hole 작업.
        hole 타입은 일반 집게 방식과 잡는 좌표, TCP, 여는 동작이 다름.
        """

        self.node.get_logger().info("hole 타입 작업 시작")

        self.grasp_hole_bottle()
        self.open_hole_lid()
        self.move_pour_hole()
        self.move_trash_hole()

        self.node.get_logger().info("hole 타입 작업 완료")

    # --------------------------------------------------
    # 12. 전체 실행 순서
    # --------------------------------------------------
    def run(self):
        self.node.get_logger().info("전체 작업 시작")

        self.wait_for_task()

        if self.lid_type == "hole":
            self.run_hole_lid_task()
        else:
            self.run_gripper_lid_task()

        self.notify_done()

        self.node.get_logger().info("전체 작업 완료")


def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node("pour_pills", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    robot = None

    try:
        robot = PourPills(node)
        robot.run()

    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt")

    except Exception as e:
        node.get_logger().error(f"Robot error: {e}")

    finally:
        if node is not None:
            node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":
    main()

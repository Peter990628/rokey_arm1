import json
from time import sleep

import rclpy
import DR_init
import requests
from rclpy.node import Node
from std_msgs.msg import String

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

VELOCITY = 50
ACC = 50
ON = 1
OFF = 0

MEDICINE_TOPIC = "/dsr01/pharmacy/medicine"
DONE_URL = "http://172.23.0.129:8000/api/tasks/refill/"

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL



class PourPills(Node):
    def __init__(self):
        super().__init__("pour_pills", namespace = ROBOT_ID)
        DR_init.__dsr__node = self

        from DSR_ROBOT2 import (
            set_digital_output,
            set_tool,
            set_tcp,
            movel,
            movejx,
            task_compliance_ctrl,
            set_desired_force,
            get_current_posx,
            release_force,
            DR_BASE,
            DR_MV_MOD_REL,
        )

        from DR_common2 import posx

        # DSR 함수 저장
        self.set_digital_output = set_digital_output
        self.set_tool = set_tool
        self.set_tcp = set_tcp
        self.movel = movel
        self.movejx = movejx
        self.task_compliance_ctrl = task_compliance_ctrl
        self.set_desired_force = set_desired_force
        self.get_current_posx = get_current_posx
        self.release_force = release_force

        # DSR 상수 저장
        self.DR_BASE = DR_BASE
        self.DR_MV_MOD_REL = DR_MV_MOD_REL
        self.posx = posx

        self.vel = VELOCITY
        self.acc = ACC

        self.current_task = None
        self.task_queue = []
        self.medicine_id = None
        self.medicine_name = None
        self.refill_amount = None
        self.lid_type = None

        self.dispensing_x = None
        self.dispensing_y = None
        self.dispensing_z = None
        self.dispensing_rx = None
        self.dispensing_ry = None
        self.dispensing_rz = None

        self.drawer_x = None
        self.drawer_y = None
        self.drawer_z = None
        self.drawer_rx = None
        self.drawer_ry = None
        self.drawer_rz = None

        self.X_DISPENSER_POS = None
        self.X_DRAWER = None
        self.X_LOCK_RETURN = None
        self.X_TRASH_DROP = None
        self.X_TWEEZER_HOME = None
        self.X_TWEEZER_NEAR = None
        self.X_DRAWER_CLOSED = None

        self.define_positions()
        self.create_subscriptions()
        self.init_robot()

    # --------------------------------------------------
    # 0. 위치 정의
    # --------------------------------------------------
    def define_positions(self):
        
        self.X_TRASH_DROP = self.posx(-423.12, -96.72, 89.41, 8.51, -179.18, 101.63)      # 쓰레기통 위치
        self.X_TWEEZER_HOME = self.posx(744.83, -5.82, 98.24, 177.66, -87.42, -81.97 )    # 집게 툴 거치대 위치
        self.X_TWEEZER_NEAR = self.posx(704.83, -5.82, 98.24, 177.66, -87.42, -81.97)    # 집게 툴 거치대 주변 위치     
    # ----------------------------
    # 구독하는거
    # ----------------------------------
    def create_subscriptions(self):
        self.sub_medicine = self.create_subscription(String, MEDICINE_TOPIC, self.dispenser_pose_callback, 10)
        self.get_logger().info(f"구독함:{MEDICINE_TOPIC}")

    def dispenser_pose_callback(self, msg):
        try:
            data = json.loads(msg.data)
            if not isinstance(data, list) or len(data) == 0:
                raise ValueError("수신 데이터는 비어있지 않은 list여야 함")

            self.task_queue.extend(data)
            self.get_logger().info(
                f"작업 {len(data)}개 추가 수신, "
                f"현재 대기 작업 {len(self.task_queue)}개"
            )

        except Exception as e:
            self.get_logger().error(f"작업데이터받기실패:{e}")

    def set_task_from_data(self, task):
        required_keys = [
            "medicine_number",
            "medicine_name",

            "dispensing_x",
            "dispensing_y",
            "dispensing_z",
            "dispensing_rx",
            "dispensing_ry",
            "dispensing_rz",

            "drawer_x",
            "drawer_y",
            "drawer_z",
            "drawer_rx",
            "drawer_ry",
            "drawer_rz",
            
            "lid_type",
            "storage_stock",
            "dispensing_stock"
        ]
        for key in required_keys:
            if key not in task:
                raise KeyError(f"{key}가 없음")
        
        self.current_task = task
        self.medicine_id = task["medicine_number"]
        self.medicine_name = task["medicine_name"]

        self.dispensing_x = float(task["dispensing_x"])
        self.dispensing_y = float(task["dispensing_y"])
        self.dispensing_z = float(task["dispensing_z"])
        self.dispensing_rx = float(task["dispensing_rx"])
        self.dispensing_ry = float(task["dispensing_ry"])
        self.dispensing_rz = float(task["dispensing_rz"])

        self.drawer_x = float(task["drawer_x"])
        self.drawer_y = float(task["drawer_y"])
        self.drawer_z = float(task["drawer_z"])
        self.drawer_rx = float(task["drawer_rx"])
        self.drawer_ry = float(task["drawer_ry"])
        self.drawer_rz = float(task["drawer_rz"])

        self.lid_type = str(task["lid_type"]).strip().lower()

        self.storage_stock = int(task["storage_stock"])
        self.dispensing_stock = int(task["dispensing_stock"])

        self.refill_amount = self.storage_stock

        if self.refill_amount <= 0:
            raise ValueError("storage_stock이 0 이하라서 리필할 수 없음")

        self.X_DISPENSER_POS = self.posx(
            self.dispensing_x,
            self.dispensing_y,
            self.dispensing_z,
            self.dispensing_rx,
            self.dispensing_ry,
            self.dispensing_rz 
        )

        self.X_DRAWER = self.posx(
            self.drawer_x, 
            self.drawer_y,
            self.drawer_z,
            self.drawer_rx,
            self.drawer_ry,
            self.drawer_rz
        )

    def wait_for_task(self):
        self.get_logger().info("작업 데이터 대기 중")

        while rclpy.ok() and len(self.task_queue) == 0:
            rclpy.spin_once(self, timeout_sec=0.1)

        if len(self.task_queue) == 0:
            raise RuntimeError("작업 데이터 수신 실패")

    # --------------------------------------------------
    # 1. 로봇 초기 세팅
    # --------------------------------------------------
    def init_robot(self):
        self.get_logger().info("로봇 초기 세팅 시작")
        self.set_tool("Tool Weight_1")
        self.set_tcp("Tool_v1") 
        self.release()
        self.get_logger().info("약 붓기 시작")

    # --------------------------------------------------
    # 2. 그리퍼
    # --------------------------------------------------
    def release(self):
        self.get_logger().info("그리퍼 열기")

        self.set_digital_output(1, OFF)
        self.set_digital_output(2, OFF)

        self.set_digital_output(1, ON)
        self.set_digital_output(2, OFF)

        sleep(1)
        
    def grip(self):
        self.get_logger().info("그리퍼 닫기")

        self.set_digital_output(1, OFF)
        self.set_digital_output(2, OFF)

        self.set_digital_output(1, OFF)
        self.set_digital_output(2, ON)

        sleep(1)
    
    def little(self):
        self.get_logger().info("그리퍼 살짝 닫기")

        self.set_digital_output(1, OFF)
        self.set_digital_output(2, OFF)

        self.set_digital_output(1, ON)
        self.set_digital_output(2, ON)

        sleep(1)

    # --------------------------------------------------
    # 5. 조제기 / 서랍 쪽으로 이동
    # --------------------------------------------------
    def open_drawer(self):
        if self.X_DRAWER is None:
            raise RuntimeError("서랍 열기 위치가 아직 설정되지 않음")

        self.get_logger().info("서랍 열기 시작")
        self.movejx(self.X_DRAWER, vel=self.vel, acc=self.acc, ref=self.DR_BASE, sol = 2 )
        self.grip()
        self.movel(self.posx(-125, 0, 0, 0, 0, 0), vel=20, acc=20, ref=self.DR_BASE, mod=self.DR_MV_MOD_REL)

        cur_pos = self.get_current_posx()[0]
        x, y, z, rx, ry, rz = cur_pos[:6]

        self.X_DRAWER_CLOSED = self.posx(x, y, z, rx, ry, rz)

        self.get_logger().info("서랍 열기 완료")
    
    def go_to_tool(self):
        self.X_LOCK_RETURN = self.posx() # 집게 잡는거 일때(나중에 없어질 예정 테스트용)
        # self.X_LOCK_RETURN = self.posx(551.25, 3.43, 0.70, 21.22, -179.25, 14.06) # 툴 잡는거 일때(나중에 없어질 예정 테스트용)

        if self.X_LOCK_RETURN is None:
            raise RuntimeError("X_LOCK_RETURN 좌표가 설정되지 않음")

        self.movejx(self.X_LOCK_RETURN, vel = self.vel, acc = self.acc, sol = 2)
        
        cur_pos = self.get_current_posx()[0]
        x, y, z, rx, ry, rz = cur_pos[:6]

        # TODO: 돌리기 전 접근 자세 보정값 실측 후 수정
        contact_pos = self.posx(x, y, z, rx, ry, rz)
        self.movel(contact_pos, vel = self.vel, acc = self.acc, ref=self.DR_BASE)

        self.movel(self.posx(0,0,30, 0, 0, 0), vel = self.vel, acc = self.acc, ref=self.DR_BASE, mod=self.DR_MV_MOD_REL)

    def move_pour(self):
        if self.X_DISPENSER_POS is None:
            raise RuntimeError("조제기 위치가 아직 설정되지 않음")

        self.get_logger().info("조제기 위치로 이동 시작")
        self.movejx(
            self.X_DISPENSER_POS,
            vel=self.vel,
            acc=self.acc,
            ref=self.DR_BASE,
            sol=2,
        )

        self.contact_dispenser_tweezer()
        self.pour_tweezer()
        self.get_logger().info("조제기 위치 작업 완료")

    # --------------------------------------------------
    # 6. 조제기 밀착 force control
    # --------------------------------------------------
    def contact_dispenser_tweezer(self):
        self.get_logger().info("조제기 밀착 제어 시작")
        self.set_tcp("집게 끝")

        cur_pos = self.get_current_posx()[0]
        x, y, z, rx, ry, rz = cur_pos[:6]

        # TODO: 밀착 전 접근 자세 보정값 실측 후 수정
        contact_pos = self.posx(x, y, z, rx, ry, rz)
        self.movel(contact_pos, vel=10, acc=10, ref=self.DR_BASE)

        try:
            self.task_compliance_ctrl([10000, 500, 10000, 10000, 10000, 10000])
            self.set_desired_force(
                fd=[0, 10, 0, 0, 0, 0],
                dir=[0, 1, 0, 0, 0, 0],
                mod=self.DR_MV_MOD_REL,
            )
            sleep(1)
        finally:
            self.release_force()
            self.get_logger().info("조제기 밀착 제어 종료")

    # --------------------------------------------------
    # 7. 약 붓기
    # --------------------------------------------------
    def pour_tweezer(self):
        cur_pos = self.get_current_posx()[0]
        x, y, z, rx, ry, rz = cur_pos[:6]

        pour_start_pose = self.posx(x, y, z, rx, ry, rz)

        pour_tilt_pose = self.posx(0, 0, 0, 20, 0, 0)

        self.movel(pour_tilt_pose, vel=10, acc=10, ref=self.DR_BASE, mod = self.DR_MV_MOD_REL)
        sleep(2)
        self.movel(pour_start_pose, vel=10, acc=10, ref=self.DR_BASE)

        self.get_logger().info("약 붓기 완료")
        
        self.notify_done()

    # --------------------------------------------------
    # 9. 약통 버리러 이동
    # --------------------------------------------------
    def move_trash(self):
        self.get_logger().info("약통 버리기 시작")
        self.movejx(self.X_TRASH_DROP, vel=self.vel, acc=self.acc, ref=self.DR_BASE, sol=2)
        self.little()
        self.get_logger().info("약통 버리기 완료")

    # --------------------------------------------------
    # 9. 집게 놓기 
    # --------------------------------------------------
    def tool_drop(self):
        self.get_logger().info("집게 놓기 시작")
        self.movejx(self.X_TWEEZER_NEAR, vel=self.vel, acc=self.acc, ref=self.DR_BASE, sol=2)
        self.movejx(self.X_TWEEZER_HOME, vel=self.vel, acc=self.acc, ref=self.DR_BASE, sol=2)
        self.release()
        self.get_logger().info("집게 놓기 완료")

    # --------------------------------------------------
    # 10. 서랍 닫기
    # --------------------------------------------------
    def close_drawer(self):
        if self.X_DRAWER_CLOSED is None:
            raise RuntimeError("서랍 열린 후 위치 X_DRAWER_CLOSED가 설정되지 않음")
        if self.X_DRAWER is None:
            raise RuntimeError("서랍 닫기 위치가 아직 설정되지 않음")

        self.get_logger().info("서랍 닫기 시작")
        self.movejx(self.X_DRAWER_CLOSED, vel=self.vel, acc=self.acc, ref=self.DR_BASE, sol=2)

        self.movel(
            self.posx(125, 0, 0, 0, 0, 0),
            vel=10,
            acc=10,
            ref=self.DR_BASE,
            mod=self.DR_MV_MOD_REL,
        )
        self.get_logger().info("서랍 닫기 완료")

    # --------------------------------------------------
    # 11. DB / task manager 알림
    # --------------------------------------------------
    def notify_done(self):

        if self.medicine_name is None:
            raise RuntimeError("medicine_name이 설정되지 않음")

        if self.refill_amount is None:
            raise RuntimeError("refill_amount가 설정되지 않음")

        self.get_logger().info("리필 완료post 함")
        payload = {
            "medicine_name": self.medicine_name,
            "amount": self.refill_amount
        }

        try:
            response = requests.post(DONE_URL, json=payload, timeout=3)
            if response.ok:
                self.get_logger().info(f"리필완 post 보냄: {response.json()}")
            else:
                self.get_logger().error(f"리필포스트보내기실패: status ={response.status_code}, body={response.text}")

        except Exception as e:
            self.get_logger().error(f"예외발생:{e}")        
            
    def run_gripper_lid_task(self):
        self.get_logger().info("집게 방식 작업 시작")

        
        self.open_drawer()
        self.set_tcp("Tool_tweezer")
        self.go_to_tool()
        self.set_tcp("집게 끝")
        self.move_pour()
        self.move_trash()
        self.set_tcp("Tool_tweezer")
        self.tool_drop()
        self.close_drawer()

        self.get_logger().info("집게 방식 작업 완료")

    def run_hole_lid_task(self):
        self.get_logger().info("pull 타입 작업 시작")

        self.open_drawer()
        self.set_tcp("Tool_pill")
        self.go_to_tool()
        self.set_tcp("Tool_pill_side")
        self.move_pour()
        self.move_trash()# 이거했을 때 약통 안버려지면 little>release로 바꿔서 함수 하나 더 만드셈
        self.close_drawer()

        self.get_logger().info("병따개 끝")


        self.get_logger().info("pull 타입 작업 완료")

    # --------------------------------------------------
    # 12. 전체 실행 순서
    # --------------------------------------------------
    def run(self):
        self.get_logger().info("전체 작업 시작")

        self.wait_for_task()

        while rclpy.ok():
            while self.task_queue:
                task = self.task_queue.pop(0)
                self.set_task_from_data(task)

                self.get_logger().info(
                    f"작업 시작: {self.medicine_name}, "
                    f"lid_type={self.lid_type}, "
                    f"refill_amount={self.refill_amount}"
                )

                if self.lid_type == "pull":
                    self.run_hole_lid_task()
                else:
                    self.run_gripper_lid_task()

                self.get_logger().info(f"작업 완료: {self.medicine_name}")

                # 약 하나 끝난 뒤 새 토픽이 들어왔는지 확인
                rclpy.spin_once(self, timeout_sec=0.1)

            # queue가 비었을 때도 새 작업을 조금 기다림
            self.get_logger().info("대기 작업 없음. 새 작업 대기 중")
            rclpy.spin_once(self, timeout_sec=0.5)


def main(args=None):
    rclpy.init(args=args)
    robot = None

    try:
        robot = PourPills()
        robot.run()

    except KeyboardInterrupt:
        if robot is not None:
            robot.get_logger().info("Keyboard Interrupt")

    except Exception as e:
        if robot is not None:
            robot.get_logger().error(f"Robot error: {e}")
        else:
            print(f"Robot init error: {e}")

    finally:
        if robot is not None:
            robot.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

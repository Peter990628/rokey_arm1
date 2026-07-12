import rclpy
import DR_init
from time import sleep
import time

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

ON, OFF = 1, 0

class SpinLidTester:
    def __init__(self, node):
        self.node = node

        from DSR_ROBOT2 import (
            set_digital_output, set_tool, set_tcp, 
            movej, movel, movejx, wait, DR_BASE, DR_MV_MOD_REL,
            set_desired_force, release_force, get_current_posx,
            task_compliance_ctrl, release_compliance_ctrl, amovel, get_tool_force 
        )
        from DR_common2 import posj, posx

        self.set_digital_output = set_digital_output
        self.set_tool = set_tool
        self.set_tcp = set_tcp
        self.movej = movej
        self.movel = movel
        self.movejx = movejx
        self.wait = wait
        
        # 2. 클래스 속성에 매핑 
        self.set_desired_force = set_desired_force 
        self.release_force = release_force         
        self.get_current_posx = get_current_posx   
        self.task_compliance_ctrl = task_compliance_ctrl
        self.release_compliance_ctrl = release_compliance_ctrl
        self.amovel = amovel              
        self.get_tool_force = get_tool_force  

        self.DR_BASE = DR_BASE
        self.DR_MV_MOD_REL = DR_MV_MOD_REL
        self.posj = posj
        self.posx = posx
        self.vel = VELOCITY
        self.acc = ACC

        self.X_LOCK_RETURN = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.define_positions()
        

    def define_positions(self):
        self.J_READY = self.posj(0, 0, 90, 0, 90, 0)
        self.X_STORAGE_APPROACH = self.posx(357.12, 219.79, 200.52, 96.00, 176.96, 105.65)
        self.storage_loc = self.posx(437.39, 423.46, 215.54, 44.38, -180.00, 50.46)
        self.X_PULL_FIX_ABOVE = self.posx(550.90, 1.02, 200.52, 8.33, -179.62, 18.10)
        self.X_PULL_FIX = self.posx(551.90, 2.2, 50.91, 8.33, -179.62, 18.10)
        self.X_OPENER_TOOL_ABOVE = self.posx(582.33, 244.77, 129.81, 39.98, 180.00, 132.74)
        self.X_OPENER_TOOL = self.posx(582.33, 244.77, 99.81, 39.98, 180.00, 132.74)
        self.X_OPEN_READY = self.posx(598.43, 5.36, 199.71, 0.66, 179.72, -0.52)        # 조금 위로 설정
        self.X_SPIN_LID_ABOVE = self.posx()     # 약통 뚜껑 위치보다 조금 위

        self.node.get_logger().info("테스트용 가상 약통 좌표(storage_loc) 세팅 완료.")


    def release(self):
        self.node.get_logger().info("[시뮬레이션] 그리퍼 열기")
        self.set_digital_output(1, OFF)
        self.set_digital_output(2, OFF)
        self.set_digital_output(1, ON)
        self.set_digital_output(2, OFF)
        sleep(0.5)

    def grip(self):
        self.node.get_logger().info("[시뮬레이션] 그리퍼 닫기 (약통 파지)")
        self.set_digital_output(1, OFF)
        self.set_digital_output(2, OFF)
        self.set_digital_output(1, OFF)
        self.set_digital_output(2, ON)
        sleep(0.5)

    def init_robot(self):
        self.node.get_logger().info("로봇 초기 세팅 및 레디 포즈 이동")
        self.set_tool("Tool Weight_1")
        self.set_tcp("Tool_v1")
        self.release()
        self.movej(self.J_READY, vel=self.vel, acc=self.acc)
        self.wait(1.0)

    # def log_current_pos(self) :
    #     current_posx, _ = self.get_current_posx(ref=self.DR_BASE)
    #     current_posj = self.get_current_posj()
    #     x = current_posx[0]
    #     y = current_posx[1]
    #     z = current_posx[2]
    #     self.node.get_logger().info(f"좌표 확인: x={x:.2f}, y={y:.2f}, z={z:.2f}")
    #     self.node.get_logger().info(f"좌표 확인(joint): j1={current_posj[0]:.2f}, j2={current_posj[1]:.2f}, j3={current_posj[2]:.2f}, j4={current_posj[3]:.2f}, j5={current_posj[4]:.2f}, j6={current_posj[5]:.2f}\n")

    def storage_grasp(self):
        self.node.get_logger().info("=== 단위 테스트 시작: 적재소 약통 파지 시퀀스 ===")

        self.release()
        
        self.node.get_logger().info("1. 약통 위 안전 고도(Approach)로 이동")
        self.movejx(self.X_STORAGE_APPROACH, vel=self.vel, acc=self.acc, ref=self.DR_BASE, sol=2)
        self.wait(0.5)
        
        self.node.get_logger().info("2. 가상의 약통 위치로 이동")
        self.movejx(self.storage_loc, vel=40, acc=40, ref=self.DR_BASE, sol=2)
        self.wait(0.5)
        
        self.node.get_logger().info("3. 툴 삽입을 위한 미세 하강 (-27mm)")
        self.movel(self.posx(0, 0, -27, 0, 0, 0), vel=40, acc=40, ref=self.DR_BASE, mod=self.DR_MV_MOD_REL)
        
        self.grip()
        
        self.node.get_logger().info("4. 약통 파지 후 안전 고도로 인출 (+40mm)")
        self.movel(self.posx(0, 0, 40, 0, 0, 0), vel=self.vel, acc=self.acc, ref=self.DR_BASE, mod=self.DR_MV_MOD_REL)
        self.wait(0.5)

        self.node.get_logger().info("5. 약통 위 안전 고도(Approach)로 이동")
        self.movejx(self.X_STORAGE_APPROACH, vel=self.vel, acc=self.acc, ref=self.DR_BASE, sol=2)
        self.wait(0.5)

        self.node.get_logger().info("=== 단위 테스트 완료 1 ===")

    def pull_down(self):
        self.node.get_logger().info("순응 제어 모드: 거치대에 약통 꽂기 시도 (이중 조건 감시)")
        
        self.task_compliance_ctrl(stx=[500, 500, 100, 5000, 5000, 5000])
        self.wait(0.5)
        
        self.set_desired_force(fd=[0, 0, -30, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0])
        self.wait(0.5)

        target_z = 29.93         
        z_tolerance = 1.0        

        while True:
            current_pos = self.get_current_posx(ref=self.DR_BASE)[0]
            # current_force = self.get_tool_force(ref=self.DR_BASE)
            
            current_z = current_pos[2]
            # current_fz = abs(current_force[2]) 
            
            condition_pos_met = abs(current_z - target_z) <= z_tolerance
            
            if condition_pos_met:
                self.node.get_logger().info(f"=> 안착 조건 100% 충족")
                self.release_compliance_ctrl()
                self.release_force()

                break

            sleep(0.02)



    def lock(self):
        self.node.get_logger().info("반시계 방향 회전을 통해 거치대 고정 시도 (Lock)")    

        self.movej(self.posj(0, 0, 0, 0, 0, -17), vel=3, acc=1, mod=self.DR_MV_MOD_REL)

        # self.task_compliance_ctrl(stx=[500, 500, 500, 5000, 5000, 100])
        # self.wait(0.5)
        
        # self.set_desired_force(fd=[0, 0, -5, 0, 0, -3], dir=[0, 0, 1, 0, 0, 1])
        # self.wait(0.5)

        # target_rz = 22.78         
        # rz_tolerance = 1.0      

        # while True:
        #     current_pos = self.get_current_posx(ref=self.DR_BASE)[0]
        #     # current_force = self.get_tool_force(ref=self.DR_BASE)
            
        #     current_z = current_pos[3]
        #     # current_fz = abs(current_force[2]) 
            
        #     condition_pos_met = abs(current_z - target_rz) <= rz_tolerance
            
        #     if condition_pos_met:
        #         self.node.get_logger().info(f"=> 잠금 조건 100% 충족")
        #         self.release_force()
        #         self.release_compliance_ctrl()
        #         self.release_force()

        #         break

        #     sleep(0.02)

    def pull_fix_lid(self):
        self.node.get_logger().info("=== 단위 테스트 시작: 당겨서 여는 약통 거치대 고정 ===")     

        # self.set_tcp('Tool_pill')
        # self.wait(3.5)
        # self.node.get_logger().info("TCP 변경 완료")
        
        self.node.get_logger().info("1. 약통 거치대 force 시작 위치의 상공으로 이동")
        self.movejx(self.X_PULL_FIX_ABOVE, vel=30, acc=30, ref=self.DR_BASE, sol=2)
        self.wait(1)

        self.node.get_logger().info("2. 약통 거치대 force 시작 위치로 이동")
        self.movejx(self.X_PULL_FIX, vel=5, acc=5, ref=self.DR_BASE, sol=2)
        self.wait(0.5)

        self.node.get_logger().info("3. 거치대 소켓에 가압 삽입 (pull_down)")
        self.pull_down()
        self.wait(0.5)

        self.node.get_logger().info("4. 반시계 방향 회전하여 락킹 (lock)")
        self.lock()
        self.wait(0.5)

        # self.log_current_pos()

        # 현재 위치 실측 저장 
        self.X_LOCK_RETURN = self.get_current_posx(ref=self.DR_BASE)[0]
        self.node.get_logger().info(f"동적 좌표 저장 완료: {self.X_LOCK_RETURN}")

        self.node.get_logger().info("4. 그리퍼 해제 후 수직 안전 탈출")
        self.release()

        self.movel(self.posx(0, 0, 100, 0, 0, 0), vel=20, acc=20, ref=self.DR_BASE, mod=self.DR_MV_MOD_REL)

        # self.log_current_pos()
        
        self.node.get_logger().info("=== 단위 테스트 완료 2 ===")

    def spin_open(self):
        
        # Z축 방향 순응 제어 활성화
        self.task_compliance_ctrl([10000, 10000, 300, 10000, 10000, 10000])

        # Z축 하향 방향(DR_BASE 기준 -Z축)으로 뚜껑을 풀기 위한 강력한 가압(30N) 조건 인가
        self.set_desired_force(fd=[0, 0, -10, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0])
        self.wait(0.5)

        # 회전
        self.amove_periodic(
            amp=[0, 0, 0, 0, 0, 12],       # Rz축 기준 ±12도 진폭 
            period=[0, 0, 0, 0, 0, 0.8],    # 0.8초 주기 (조금 더 기민하게 진동)
            atime=0.4, 
            repeat=2,                       # 왕복 2회 회전 털기 
            ref=self.DR_BASE
        )
        self.wait(0.2)

        self.movel(
            self.posx(0, 0, 10, 0, 0, -140), 
            vel=15, acc=15, 
            ref=self.DR_BASE, 
            mod=self.DR_MV_MOD_REL
        )
    
        # 힘 제어 복구
        self.release_force()
        self.wait(0.5)
        

        self.node.get_logger().info("약통 뚜껑 열기 완료")

    
    def spin_lid(self):
        
        """

        1. 약통 꽂힌 위치의 위로 이동
        2. 그리퍼 닫기
        3. 반시계 방향으로 회전시켜 열기
        4. 뚜껑 버리기
        
        """

        self.node.get_logger().info("돌려서 약통 뚜껑 열기 시작")

        # 약통 꽂힌 위치의 위로 이동
        self.movejx(
            self.X_SPIN_LID_ABOVE, vel=self.vel, acc=self.acc, ref=self.DR_BASE, sol=2
        )

        self.movel(
            self.posx(0, 0, 0, 0, 0, -30), 
            vel=15, acc=15, 
            ref=self.DR_BASE, 
            mod=self.DR_MV_MOD_REL
        )

        # 그리퍼 닫기
        self.grip()

        # 반시계 방향으로 회전시켜 열기
        self.spin_open()

        # 뚜껑 버리기
        self.trash()



def main(args=None):
    rclpy.init(args=args)
    
    # 1. 노드 생성 및 두산 전역 변수 매핑
    node = rclpy.create_node("storage_grasp_tester", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    tester = SpinLidTester(node)

    for i in range(5, 0, -1):
        node.get_logger().info(f"{i}초 후 시작...")
        sleep(1.0)

    try:
        # 3. 별도의 스레드나 spin 없이 순차적으로 직관적인 구동
        tester.init_robot()
        tester.storage_grasp()
        # tester.set_tcp('Tool_pill')
        # tester.spin_fix_lid()
        tester.spin_lid()

        
    except KeyboardInterrupt:
        node.get_logger().info("테스트 강제 종료")
    except Exception as e:
        node.get_logger().error(f"로봇 에러 발생: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
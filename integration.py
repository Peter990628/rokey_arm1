import rclpy
import DR_init
from time import sleep
import time
# import threading
from rclpy.node import Node
from std_msgs.msg import String
import json
import math
import requests

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

# --------------------------------------------------
# [변수 정의 1] opener.py 관련 (Suffix _1 적용)
# --------------------------------------------------
VELOCITY_1, ACC_1 = 50, 50

POUR_TOTAL_ANGLE_DEG_1 = -30.0
POUR_STEP_DEG_1 = 6.0
POUR_VEL_1 = 5
POUR_ACC_1 = 5
POUR_HOLD_SEC_1 = 1.0

# --------------------------------------------------
# [변수 정의 2] pour_pills_rotate.py 관련 (Suffix _2 적용)
# --------------------------------------------------
VELOCITY_2, ACC_2 = 50, 50

POUR_TOTAL_ANGLE_DEG_2 = 60.0
POUR_STEP_DEG_2 = 10.0
POUR_VEL_2 = 15
POUR_ACC_2 = 15
POUR_HOLD_SEC_2 = 1.0

MEDICINE_LIFT_DISTANCE_2 = 40.0
COMPLIANCE_STX_2 = [10000, 10000, 700, 300, 300, 300]
DESIRED_FORCE_2 = [0, 0, 50, 0, 0, 0]
FORCE_DIRECTION_2 = [0, 0, 1, 0, 0, 0]
FORCE_CHECK_INTERVAL_SEC_2 = 0.05

DRAWER_SAFE_JOINT_2 = [-42.63, -0.65, 99.42, -1.37, 81.22, 7.57]

BOTTLE_GRIP_INPUT_2 = 1
BOTTLE_GRIP_OK_VALUE_2 = 1
BOTTLE_GRIP_TIMEOUT_SEC_2 = 2.0
BOTTLE_GRIP_CHECK_INTERVAL_SEC_2 = 0.05

ON, OFF = 1, 0

TEST_MEDICINE_NUMBER = 3

TEST_TASKS = {
    1: {
        "medicine_number": 1,
        "medicine_name": "탁센",
        "storage_x": 218.23,
        "storage_y": 295.72,
        "storage_z": 251.97,
        "storage_rx": 51.17,
        "storage_ry": 178.87,
        "storage_rz": -116.43,
        "dispensing_x": -15.23,
        "dispensing_y": 23.85,
        "dispensing_z": 48.29,
        "dispensing_rx": 14.40,
        "dispensing_ry": 55.66,
        "dispensing_rz": -108.35,
        "bottle_tip_offset_x": 0.0,
        "bottle_tip_offset_y": 24.0,
        "bottle_tip_offset_z": -45.0,
        "drawer_x": -20.87,
        "drawer_y": 10.92,
        "drawer_z": 110.58,
        "drawer_rx": -38.71,
        "drawer_ry": -37.23,
        "drawer_rz": -58.32,
    },
    2: {
        "medicine_number": 2,
        "medicine_name": "타이레놀",
        "storage_x": 365.66,
        "storage_y": 304.19,
        "storage_z": 257.04,
        "storage_rx": 42.71,
        "storage_ry": 178.99,
        "storage_rz": 43.13,
        "dispensing_x": -23.23,
        "dispensing_y": 25.53,
        "dispensing_z": 45.30,
        "dispensing_rx": 20.25,
        "dispensing_ry": 55.52,
        "dispensing_rz": -120.31,
        "bottle_tip_offset_x": 0.0,
        "bottle_tip_offset_y": 23.5,
        "bottle_tip_offset_z": -51.0,
        "drawer_x": -29.61,
        "drawer_y": 13.50,
        "drawer_z": 104.92,
        "drawer_rx": -52.24,
        "drawer_ry": -39.23,
        "drawer_rz": -44.14,
    },
    3: {
        "medicine_number": 3,
        "medicine_name": "별사탕",
        "storage_x": 437.39,
        "storage_y": 423.46,
        "storage_z": 215.54,
        "storage_rx": 44.38,
        "storage_ry": -180.0,
        "storage_rz": 50.46,
        "dispensing_x": -33.32,
        "dispensing_y": 29.23,
        "dispensing_z": 41.55,
        "dispensing_rx": 34.95,
        "dispensing_ry": 62.80,
        "dispensing_rz": -132.18,
        "bottle_tip_offset_x": 0.0,
        "bottle_tip_offset_y": 25.0,
        "bottle_tip_offset_z": -42.0,
        "drawer_x": -38.18,
        "drawer_y": 18.52,
        "drawer_z": 99.15,
        "drawer_rx": -59.93,
        "drawer_ry": -46.03,
        "drawer_rz": -40.83,
    },
}

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# --------------------------------------------------
# [새로 클래스 추가] pour_pills_rotate.py의 예외 클래스
# --------------------------------------------------
class GraspError(RuntimeError):
    """약통 파지 관련 기본 예외."""
    pass

class GraspTimeoutError(GraspError):
    """약통을 잡지 못한 경우. 재시도 가능."""
    pass

class GraspSensorError(GraspError):
    """파지 센서 읽기 오류. 재시도 불가능."""
    pass


class PharmacyRobot(Node):
    def __init__(self):
        super().__init__('pharmacy_robot')
        DR_init.__dsr__node = self

        from DSR_ROBOT2 import (
            set_digital_output, get_digital_input, set_tool, set_tcp,
            movej, movel, movejx, wait, trans, task_compliance_ctrl,
            get_tool_force, amove_periodic, check_position_condition,
            set_desired_force, get_current_posx, release_force,
            release_compliance_ctrl, amovel, amovej, stop, check_motion, 
            DR_TOOL, DR_BASE, DR_MV_MOD_ABS, DR_MV_MOD_REL, DR_AXIS_Z, 
            DR_SSTOP, DR_QSTOP, DR_FC_MOD_REL
        )
        from DR_common2 import posj, posx

        self.set_digital_output = set_digital_output
        self.get_digital_input = get_digital_input
        self.set_tool = set_tool
        self.set_tcp = set_tcp
        self.movej = movej
        self.movel = movel
        self.movejx = movejx 
        self.wait = wait
        self.trans = trans
        self.task_compliance_ctrl = task_compliance_ctrl
        self.get_tool_force = get_tool_force
        # self.amove_periodic = amove_periodic
        self.check_position_condition = check_position_condition
        self.set_desired_force = set_desired_force
        self.get_current_posx = get_current_posx
        self.release_force = release_force
        self.release_compliance_ctrl = release_compliance_ctrl
        self.check_motion = check_motion

        self.DR_BASE = DR_BASE
        self.DR_MV_MOD_ABS = DR_MV_MOD_ABS
        self.DR_MV_MOD_REL = DR_MV_MOD_REL
        self.DR_FC_MOD_REL = DR_FC_MOD_REL
        self.DR_TOOL = DR_TOOL
        self.DR_AXIS_Z = DR_AXIS_Z
        
        self.posj = posj
        self.posx = posx
        
        self.vel_1, self.acc_1 = VELOCITY_1, ACC_1
        self.vel_2, self.acc_2 = VELOCITY_2, ACC_2

        # 상태 변수들
        self.X_LOCK_RETURN = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.bottle_tip_offset_tcp_1= [0.0, 0.0, 75.0]
        self.lid_type = None
        self.medicine_name = None

        # --------------------------------------------------
        # [새로 변수 정의] 두 코드의 좌표 데이터를 받을 수 있는 통합 변수
        # --------------------------------------------------
        self.storage_loc = None
        self.X_DISPENSER_POS = None
        self.X_DRAWER = None

        self.define_positions()
        
        # Subscriber 통합 설정
        self.create_subscription(String, "/dsr01/pharmacy/refill_required_medicine", self.medicine_callback, 10)
        self.DONE_URL = "http://172.23.0.129:8000/api/tasks/refill/"
        self.init_robot()

    def define_positions(self):
        self.J_READY = self.posj(0, 0, 90, 0, 90, 0)
        self.X_STORAGE_APPROACH = self.posx(357.12, 219.79, 200.52, 96.00, 176.96, 105.65)
        self.X_FIX_ABOVE = self.posx(550.90, 1.02, 200.52, 8.33, -179.62, 18.10)
        self.X_FIX = self.posx(551.90, 2.2, 50.91, 8.33, -179.62, 18.10)
        self.X_SPIN_LID_ABOVE = self.posx(551.90, 2.2, 100.91, 8.33, -179.62, 18.10)
        self.X_OPENER_TOOL_ABOVE = self.posx(582.33, 244.77, 129.81, 39.98, 180.00, 132.74)
        self.X_OPENER_TOOL = self.posx(582.33, 244.77, 99.81, 39.98, 180.00, 132.74)
        self.X_OPEN_READY_ABOVE = self.posx(602.07, 6.05, 182.47, 161.74, -179.88, 157.58)        
        self.X_OPEN_READY = self.posx(602.07, 6.05, 148.47, 161.74, -179.88, 157.58)        
        self.X_OPEN_LOC = self.posx(557.46, 5.58, 146.22, 104.54, -179.97, 100.42)
        self.X_TRASH = self.posx(-423.12, -96.72, 89.41, 8.51, -179.18, 101.63)

    # ------------------ 벡터 연산 헬퍼 (공통) ------------------
    @staticmethod
    def _matmul_3x3(left, right):
        return [
            [
                sum(left[row][k] * right[k][col] for k in range(3))
                for col in range(3)
            ]
            for row in range(3)
        ]

    @staticmethod
    def _matvec_3x3(matrix, vector):
        return [
            sum(matrix[row][col] * vector[col] for col in range(3))
            for row in range(3)
        ]

    @staticmethod
    def _rotation_x(angle_rad):
        c = math.cos(angle_rad)
        s = math.sin(angle_rad)
        return [
            [1.0, 0.0, 0.0],
            [0.0, c, -s],
            [0.0, s, c],
        ]

    @staticmethod
    def _rotation_y(angle_rad):
        c = math.cos(angle_rad)
        s = math.sin(angle_rad)
        return [
            [c, 0.0, s],
            [0.0, 1.0, 0.0],
            [-s, 0.0, c],
        ]

    @staticmethod
    def _rotation_z(angle_rad):
        c = math.cos(angle_rad)
        s = math.sin(angle_rad)
        return [
            [c, -s, 0.0],
            [s, c, 0.0],
            [0.0, 0.0, 1.0],
        ]

    @classmethod
    def _zyz_to_rotation_matrix(cls, a_deg, b_deg, c_deg):
        rz_a = cls._rotation_z(math.radians(a_deg))
        ry_b = cls._rotation_y(math.radians(b_deg))
        rz_c = cls._rotation_z(math.radians(c_deg))
        return cls._matmul_3x3(
            cls._matmul_3x3(rz_a, ry_b),
            rz_c,
        )

    @staticmethod
    def _angle_near_reference(angle_deg, reference_deg):
        return angle_deg + 360.0 * round(
            (reference_deg - angle_deg) / 360.0
        )

    @classmethod
    def _rotation_matrix_to_zyz(cls, rotation, reference_abc):
        r22 = max(-1.0, min(1.0, rotation[2][2]))
        b = math.acos(r22)
        sin_b = math.sin(b)
        epsilon = 1e-9

        if abs(sin_b) > epsilon:
            a = math.atan2(rotation[1][2], rotation[0][2])
            c = math.atan2(rotation[2][1], -rotation[2][0])
        elif r22 > 0.0:
            b = 0.0
            a = math.atan2(rotation[1][0], rotation[0][0])
            c = 0.0
        else:
            b = math.pi
            a = math.atan2(-rotation[1][0], -rotation[0][0])
            c = 0.0

        candidate_1 = [
            math.degrees(a),
            math.degrees(b),
            math.degrees(c),
        ]

        candidate_2 = [
            candidate_1[0] + 180.0,
            -candidate_1[1],
            candidate_1[2] + 180.0,
        ]

        candidates = []
        for candidate in (candidate_1, candidate_2):
            adjusted = [
                cls._angle_near_reference(
                    candidate[index],
                    reference_abc[index],
                )
                for index in range(3)
            ]

            score = sum(
                (adjusted[index] - reference_abc[index]) ** 2
                for index in range(3)
            )
            candidates.append((score, adjusted))

        return min(candidates, key=lambda item: item[0])[1]

    @staticmethod
    def _make_angle_sequence(total_angle_deg, step_deg):
        if step_deg <= 0.0:
            raise ValueError("POUR_STEP_DEG는 0보다 커야 함")

        if abs(total_angle_deg) < 1e-9:
            return []

        direction = 1.0 if total_angle_deg > 0.0 else -1.0
        signed_step = abs(step_deg) * direction

        angles = []
        current_angle = signed_step

        while abs(current_angle) < abs(total_angle_deg):
            angles.append(current_angle)
            current_angle += signed_step

        angles.append(total_angle_deg)
        return angles

    def _calculate_virtual_tcp_pose_1(
        self,
        start_rotation,
        tip_position_base,
        angle_deg,
        reference_abc,
    ):
        local_y_rotation = self._rotation_y(
            math.radians(angle_deg)
        )

        target_rotation = self._matmul_3x3(
            start_rotation,
            local_y_rotation,
        )

        rotated_tip_offset_base = self._matvec_3x3(
            target_rotation,
            self.bottle_tip_offset_tcp,
        )

        target_position = [
            tip_position_base[index]
            - rotated_tip_offset_base[index]
            for index in range(3)
        ]

        target_abc_1 = self._rotation_matrix_to_zyz(
            target_rotation,
            reference_abc,
        )

        target_values_1 = target_position + target_abc_1
        target_pose_1 = self.posx(*target_values_1)

        return target_pose_1, target_values_1, target_abc_1
    
    def _calculate_virtual_tcp_pose_2(
        self,
        start_rotation,
        tip_position_base,
        angle_deg,
        reference_abc,
    ):
        local_x_rotation = self._rotation_x(
            math.radians(angle_deg)
        )

        target_rotation = self._matmul_3x3(
            start_rotation,
            local_x_rotation,
        )

        rotated_tip_offset_base = self._matvec_3x3(
            target_rotation,
            self.bottle_tip_offset_tcp,
        )

        target_position = [
            tip_position_base[index]
            - rotated_tip_offset_base[index]
            for index in range(3)
        ]

        target_abc_2 = self._rotation_matrix_to_zyz(
            target_rotation,
            reference_abc,
        )

        target_values_2 = target_position + target_abc_2
        target_pose_2 = self.posx(*target_values_2)

        return target_pose_2, target_values_2, target_abc_2


    # ------------------ 기본 제어 (공통) ------------------
    def release(self):
        self.get_logger().info("그리퍼 열기")
        self.set_digital_output(1, OFF)
        self.set_digital_output(2, OFF)
        sleep(0.2)
        self.set_digital_output(1, ON)
        self.set_digital_output(2, OFF)
        sleep(1)

    def grip(self):
        self.get_logger().info("그리퍼 닫기")
        self.set_digital_output(1, OFF)
        self.set_digital_output(2, OFF)
        sleep(0.2)
        self.set_digital_output(1, OFF)
        self.set_digital_output(2, ON)
        sleep(1)
        
    def check_grasp_success(self):
        self.wait(0.5)
        current_force = self.get_tool_force(ref=self.DR_TOOL)
        if abs(current_force[2]) < 1.0: 
            return False
        return True

    def init_robot(self):
        self.get_logger().info("로봇 초기 세팅 시작")
        self.set_tool("Tool Weight_1")
        self.set_tcp("Tool_v1")
        self.release()
        self.movej(self.J_READY, vel=self.vel_1, acc=self.acc_1)
        self.get_logger().info("로봇 초기 세팅 및 레디 포즈 이동 완료")

    # ------------------ 콜백 통신 ------------------
    def medicine_callback(self, msg):
        try:
            data = json.loads(msg.data)
            
            # [통합/추가된 로직] opener와 pour_pills의 JSON 규격(List/Dict) 및 필드 차이 대응
            if isinstance(data, list):
                if len(data) == 0: raise ValueError("수신 데이터가 비어 있음")
                task_data = data[0]
            else:
                task_data = data

            self.medicine_name = task_data.get('medicine_name', 'Unknown')
            self.lid_type = task_data.get('lid_type', '').strip().lower()

            # Opener 위치 (storage_loc) 설정
            self.storage_loc = self.posx(
                float(task_data.get('storage_x', 0.0)),
                float(task_data.get('storage_y', 0.0)),
                float(task_data.get('storage_z', 0.0)),
                float(task_data.get('storage_rx', task_data.get('dispensing_x', 0.0))), 
                float(task_data.get('storage_ry', task_data.get('dispensing_y', 0.0))), 
                float(task_data.get('storage_rz', task_data.get('dispensing_z', 0.0)))
            )

            # Pourer 위치 (조제기 좌표) 설정
            self.X_DISPENSER_POS = self.posj(
                float(task_data.get('dispensing_x', 0.0)),
                float(task_data.get('dispensing_y', 0.0)),
                float(task_data.get('dispensing_z', 0.0)),
                float(task_data.get('dispensing_rx', 0.0)),
                float(task_data.get('dispensing_ry', 0.0)),
                float(task_data.get('dispensing_rz', 0.0))
            )

            self.bottle_tip_offset_tcp = [
                float(task_data.get("bottle_tip_offset_x", 0.0)),
                float(task_data.get("bottle_tip_offset_y", 0.0)),
                float(task_data.get("bottle_tip_offset_z", 75.0)),
            ]

            self.drawer_x = float(task_data.get["drawer_x"])
            self.drawer_y = float(task_data.get["drawer_y"])
            self.drawer_z = float(task_data.get["drawer_z"])
            self.drawer_rx = float(task_data.get["drawer_rx"])
            self.drawer_ry = float(task_data.get["drawer_ry"])
            self.drawer_rz = float(task_data.get["drawer_rz"])
            self.X_DRAWER = self.posj(
                self.drawer_x,
                self.drawer_y,
                self.drawer_z,
                self.drawer_rx,
                self.drawer_ry,
                self.drawer_rz,
            )

            self.lid_type = str(task_data.get["lid_type"]).strip().lower()
            self.storage_stock = int(task_data.get["storage_stock"])
            self.dispensing_stock = int(task_data.get["dispensing_stock"])

            # 현재 코드는 저장 약품 전부를 리필량으로 사용
            self.refill_amount = self.storage_stock
            if self.refill_amount <= 0:
                raise ValueError("storage_stock이 0 이하라 리필할 수 없음")

            self.get_logger().info(f"수신 성공! : {self.medicine_name} 의 리필 과정을 시작합니다. (뚜껑 타입: {self.lid_type})")
            
            # 메인 스레드 블로킹 방지를 위한 비동기 처리
            # threading.Thread(target=self.run, daemon=True).start()
        
        except Exception as e:
            self.get_logger().error(f"Callback JSON Parsing Error: {e}")

    # ==================================================
    # [Opener 로직 파트] 
    # ==================================================
    def storage_grasp(self):
        self.get_logger().info("=== 적재소 약통 파지 시퀀스 ===")
        self.movejx(self.X_STORAGE_APPROACH, vel=self.vel_1, acc=self.acc_1, ref=self.DR_BASE, sol=2)
        self.movejx(self.storage_loc, vel=self.vel_1, acc=self.acc_1, ref=self.DR_BASE, sol=2)
        self.movel(self.posx(0, 0, -27, 0, 0, 0), vel=20, acc=20, ref=self.DR_BASE, mod=self.DR_MV_MOD_REL)
        self.grip()
        self.movel(self.posx(0, 0, 40, 0, 0, 0), vel=20, acc=20, ref=self.DR_BASE, mod=self.DR_MV_MOD_REL)
        self.wait(0.5)
        self.movejx(self.X_STORAGE_APPROACH, vel=self.vel_1, acc=self.acc_1, ref=self.DR_BASE, sol=2)
        self.wait(0.5)

    def pull_down(self):
        self.get_logger().info("거치대에 약통 꽂기 시도")
        self.task_compliance_ctrl(stx=[500, 500, 100, 5000, 5000, 5000])
        self.wait(0.5)
        self.set_desired_force(fd=[0, 0, -35, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0])
        self.wait(0.5)

        target_z = 29.93         
        z_tolerance = 1.0    
        start_time = time.time()    

        while True:
            if time.time() - start_time > 5.0:
                self.get_logger().error("<Timeout> 거치대 상태를 확인하세요.")
                self.release_compliance_ctrl()
                self.release_force()
                self.movel(self.posx(0, 0, 50, 0, 0, 0), vel=20, acc=20, ref=self.DR_BASE, mod=self.DR_MV_MOD_REL)
                return False
            
            current_z = self.get_current_posx(ref=self.DR_BASE)[0][2]
            if abs(current_z - target_z) <= z_tolerance:
                self.get_logger().info("=> 안착 조건 충족")
                self.release_compliance_ctrl()
                self.release_force()
                return True
            sleep(0.02)

    def lock(self):
        self.get_logger().info("반시계 방향 회전 거치대 고정 (Lock)")    
        self.amovej(self.posj(0, 0, 0, 0, 0, -17), vel=3, acc=1, mod=self.DR_MV_MOD_REL)
        torque_threshold = 10.0 
        force_threshold = 30.0   

        while self.check_motion() != 0:
            current_force = self.get_tool_force(ref=self.DR_TOOL)
            if abs(current_force[5]) > torque_threshold or abs(current_force[0]) > force_threshold or abs(current_force[1]) > force_threshold:
                self.stop(self.DR_QSTOP)
                self.get_logger().error("락킹 중 비정상적인 끼임 감지! 즉시 회전을 중단합니다.")
                return False  
            sleep(0.01)

        self.wait(0.5)
        return True

    def fix_lid(self):
        self.get_logger().info("=== 약통 거치대 고정 시퀀스 ===")     
        self.movejx(self.X_FIX_ABOVE, vel=30, acc=30, ref=self.DR_BASE, sol=2)
        self.movejx(self.X_FIX, vel=5, acc=5, ref=self.DR_BASE, sol=2)
        self.wait(0.5)
        if not self.pull_down():
            return False
        self.wait(0.5)
        if not self.lock():
            return False
        self.wait(0.5)
        self.X_LOCK_RETURN = self.get_current_posx(ref=self.DR_BASE)[0]
        self.release()
        self.movel(self.posx(0, 0, 100, 0, 0, 0), vel=20, acc=20, ref=self.DR_BASE, mod=self.DR_MV_MOD_REL)
        return True

    def pull_lid_open(self):
        self.get_logger().info("X_OPEN_READY_ABOVE 위치로 이동")
        self.movejx(self.X_OPEN_READY_ABOVE, vel=10, acc=10, ref=self.DR_BASE, sol=2)
        self.movel(self.X_OPEN_READY, vel=5, acc=5, ref=self.DR_BASE)

        found_contact = False
        self.get_logger().info("X축 방향 탐색 시작")
        for _ in range(100):
            current_force = self.get_tool_force(ref=self.DR_BASE)
            if current_force[0] > 2.0:  
                found_contact = True
                self.get_logger().info(f"X축 충돌(뚜껑) 감지 ({current_force[0]:.2f}N)")
                break
            self.movel(self.posx(-2.0, 0, 0, 0, 0, 0), vel=5, acc=5, ref=self.DR_BASE, mod=self.DR_MV_MOD_REL)
            sleep(0.05)

        if not found_contact:
            self.get_logger().error("[예외] 뚜껑 탐색 실패. 약통이 없습니다.")
            return False 
        return True
    
    def opener_tweezer(self):
        self.grip()
        start_pose = self.get_current_pos_base()
        start_rotation = self._zyz_to_rotation_matrix(*start_pose[3:6])
        tip_offset_base = self._matvec_3x3(start_rotation, self.bottle_tip_offset_tcp)
        tip_position_base = [start_pose[i] + tip_offset_base[i] for i in range(3)]

        angles = self._make_angle_sequence(POUR_TOTAL_ANGLE_DEG_1, POUR_STEP_DEG_1)
        reference_abc = start_pose[3:6]

        for angle_deg in angles:
            target_pose, _, reference_abc = self._calculate_virtual_tcp_pose(
                start_rotation, tip_position_base, angle_deg, reference_abc)
            self.movel(target_pose, vel=POUR_VEL_1, acc=POUR_ACC_1, ref=self.DR_BASE)
        sleep(POUR_HOLD_SEC_1)
        return True

    def _return_opener_safely(self):
        self.movejx(self.X_OPENER_RETURN, vel=self.vel_1, acc=self.acc_1, ref=self.DR_BASE, sol=0)
        self.movel(self.posx(0, 0, -100, 0, 0, 0), vel=15, acc=15, ref=self.DR_BASE, mod=self.DR_MV_MOD_REL)
        self.wait(0.5)
        self.release()
        self.movel(self.posx(0, 0, 100, 0, 0, 0), vel=30, acc=30, ref=self.DR_BASE, mod=self.DR_MV_MOD_REL)

    def pull_lid(self):
        self.get_logger().info("=== 당겨서 여는 약통 뚜껑 열기 시퀀스 ===")
        self.movejx(self.X_OPENER_TOOL_ABOVE, vel=30, acc=30, ref=self.DR_BASE, sol=2)
        self.movel(self.posx(0, 0, -30, 0, 0, 0), vel=30, acc=30, ref=self.DR_BASE, mod=self.DR_MV_MOD_REL)
        
        self.grip()
        if not self.check_grasp_success():
            self.get_logger().error("툴 파지 실패!")
            self.release()
            self.movel(self.posx(0, 0, 100, 0, 0, 0), vel=30, acc=30, ref=self.DR_BASE, mod=self.DR_MV_MOD_REL)
            return False
        
        self.movel(self.posx(0, 0, 130, 0, 0, 0), vel=30, acc=30, ref=self.DR_BASE, mod=self.DR_MV_MOD_REL)
        self.X_OPENER_RETURN = self.get_current_posx(ref=self.DR_BASE)[0]
        
        if not self.pull_lid_open():
            self._return_opener_safely()
            return False

        if not self.opener_tweezer():
            self._return_opener_safely()
            return False

        self.movel(self.posx(0, 0, -150, 0, 0, 0), vel=60, acc=60, ref=self.DR_TOOL, mod=self.DR_MV_MOD_REL)
        self._return_opener_safely()
        return True

    def spin_open(self):
        self.get_logger().info("뚜껑 열기 모션 시작")
        self.task_compliance_ctrl([10000, 10000, 300, 10000, 10000, 10000])
        self.set_desired_force(fd=[0, 0, -20, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0])
        self.wait(0.5)

        for _ in range(3):
            self.movel(self.posx(0, 0, 0, 0, 0, -10), vel=50, acc=50, ref=self.DR_TOOL, mod=self.DR_MV_MOD_REL)
            self.wait(0.1) 
            
        max_attempts = 5
        for attempt in range(max_attempts):
            self.get_logger().info(f"[{attempt + 1}/{max_attempts}] 뚜껑 회전 시도 중...")
            for _ in range(2):
                self.movel(self.posx(0, 0, 0, 0, 0, -90), vel=25, acc=25, ref=self.DR_TOOL, mod=self.DR_MV_MOD_REL)
            self.release()
            self.wait(0.3)
            
            for _ in range(2):
                self.movel(self.posx(0, 0, 0, 0, 0, 90), vel=30, acc=30, ref=self.DR_TOOL, mod=self.DR_MV_MOD_REL)
                self.movel(self.posx(0, 0, -1, 0, 0, 0), vel=30, acc=30, ref=self.DR_TOOL, mod=self.DR_MV_MOD_REL)
        
            self.movel(self.posx(0, 0, 3, 0, 0, 0), vel=25, acc=25, ref=self.DR_TOOL, mod=self.DR_MV_MOD_REL)
            self.grip()
            self.wait(0.5)
            
            self.release_force()
            self.release_compliance_ctrl()
            self.wait(0.5)

            is_opened = True
            moved_z = 0
            for _ in range(1):
                self.movel(self.posx(0, 0, -1, 0, 0, 0), vel=15, acc=15, ref=self.DR_TOOL, mod=self.DR_MV_MOD_REL)
                moved_z += 1
                if abs(self.get_tool_force(ref=self.DR_TOOL)[2]) > 3.0:
                    is_opened = False
                    break
            
            if is_opened:
                self.get_logger().info("뚜껑 분리 성공!")
                self.movel(self.posx(0, 0, -40, 0, 0, 0), vel=20, acc=20, ref=self.DR_TOOL, mod=self.DR_MV_MOD_REL)
                break
            else:
                self.release()
                self.movel(self.posx(0, 0, moved_z, 0, 0, 0), vel=15, acc=15, ref=self.DR_TOOL, mod=self.DR_MV_MOD_REL)
                self.grip()
                self.task_compliance_ctrl([10000, 10000, 300, 10000, 10000, 10000])
                self.set_desired_force(fd=[0, 0, -20, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0])
                self.wait(0.5)
                self.movel(self.posx(0, 0, 5, 0, 0, 0), vel=15, acc=15, ref=self.DR_TOOL, mod=self.DR_MV_MOD_REL)
                self.wait(0.5)
        else:
            self.get_logger().warn("최대 시도 횟수 초과. 공정을 중단합니다.")

    def spin_lid(self):
        self.get_logger().info("=== 돌려서 약통 뚜껑 열기 시퀀스 ===")
        self.movejx(self.X_SPIN_LID_ABOVE, vel=self.vel_1, acc=self.acc_1, ref=self.DR_BASE, sol=2)
        self.movel(self.posx(0, 0, -35, 0, 0, 0), vel=15, acc=15, ref=self.DR_BASE, mod=self.DR_MV_MOD_REL)

        self.grip()
        if not self.check_grasp_success():
            self.get_logger().error("[예외] 뚜껑 파지 실패! 회전을 취소합니다.")
            self.release()
            self.movel(self.posx(0, 0, 50, 0, 0, 0), vel=20, acc=20, ref=self.DR_BASE, mod=self.DR_MV_MOD_REL)
            return False

        self.spin_open()
        return True

    def trash(self):
        self.movejx(self.X_TRASH, vel=self.vel_1, acc=self.acc_1, ref=self.DR_BASE, sol=2)
        self.release()
        self.get_logger().info("약통 뚜껑 버리기 완료")


    # ==================================================
    # [Pourer 로직 파트] 
    # ==================================================
    def wait_for_bottle_grip(self, timeout_sec=BOTTLE_GRIP_TIMEOUT_SEC_2):
        start_time = time.monotonic()
        while rclpy.ok():
            input_value = self.get_digital_input(BOTTLE_GRIP_INPUT_2)
            if input_value is None:
                raise GraspSensorError(f"약통 파지 입력이 None으로 반환됨: DI={BOTTLE_GRIP_INPUT_2}")
            if input_value not in (OFF, ON):
                raise GraspSensorError(f"약통 파지 입력값이 올바르지 않음: DI={BOTTLE_GRIP_INPUT_2}, value={input_value}")
            
            if input_value == BOTTLE_GRIP_OK_VALUE_2:
                elapsed = time.monotonic() - start_time
                self.get_logger().info(f"약통 파지 신호 확인 완료: DI={BOTTLE_GRIP_INPUT_2}, elapsed={elapsed:.2f} sec")
                return
                
            elapsed = time.monotonic() - start_time
            if elapsed >= timeout_sec:
                raise GraspTimeoutError(f"약통 파지 신호 시간 초과: timeout={timeout_sec:.1f} sec")
            sleep(BOTTLE_GRIP_CHECK_INTERVAL_SEC_2)
            
        raise GraspError("ROS2 종료로 약통 파지 확인이 중단됨")

    def grip_bottle_until_success(self):
        attempt = 0
        self.release()
        sleep(0.5)  

        while rclpy.ok():
            attempt += 1
            self.get_logger().info(f"{self.medicine_name} 약통 파지 시도: {attempt}회")

            try:
                # [새로 주석 추가] 쓰레기통(X_TRASH)에서 바로 X_LOCK_RETURN으로 이동 시 충돌 방지를 위해 X_FIX_ABOVE 경유
                self.movejx(self.X_FIX_ABOVE, vel=self.vel_2, acc=self.acc_2, sol=2)
                
                # 약통 보관 위치로 다시 이동
                self.movejx(self.X_LOCK_RETURN, vel=self.vel_2, acc=self.acc_2, sol=2)
                sleep(0.5)

                self.grip()
                self.wait_for_bottle_grip(timeout_sec=BOTTLE_GRIP_TIMEOUT_SEC_2)
                self.get_logger().info(f"{self.medicine_name} 약통 파지 성공: attempt={attempt}")
                return

            except GraspTimeoutError as e:
                self.get_logger().warning(f"{self.medicine_name} 약통 파지 실패: error={e}")
                try:
                    self.release()
                except Exception as release_error:
                    raise GraspError("파지 실패 후 그리퍼를 열 수 없어 재시도를 중단함") from release_error
                sleep(0.5)
                
        raise GraspError("ROS2 종료로 약통 파지 재시도가 중단됨")

    def lift_medicine_with_force(self):
        start_pose = self.get_current_pos_base()
        start_z = start_pose[2]
        self.get_logger().info(f"약통 힘제어 상승 시작: target={MEDICINE_LIFT_DISTANCE_2:.1f} mm")

        compliance_started = False
        force_started = False
        try:
            self.task_compliance_ctrl(stx=COMPLIANCE_STX_2)
            compliance_started = True
            self.wait(0.5)
            
            self.set_desired_force(fd=DESIRED_FORCE_2, dir=FORCE_DIRECTION_2, mod=self.DR_FC_MOD_REL)
            force_started = True
            self.wait(0.5)

            while rclpy.ok():
                current_z = self.get_current_pos_base()[2]
                lifted_distance = current_z - start_z
                if lifted_distance >= MEDICINE_LIFT_DISTANCE_2:
                    self.get_logger().info("목표 상승 거리 도달. 힘제어를 종료함")
                    break
                sleep(FORCE_CHECK_INTERVAL_SEC_2)

        finally:
            if force_started:
                self.release_force()
            if compliance_started:
                self.release_compliance_ctrl()

    def pick_medicine(self):
        if self.X_LOCK_RETURN is None or self.X_LOCK_RETURN == self.posx(551.24, -0.57, 32.32, 55.01, -177.89, 42.86):
            raise RuntimeError("약 보관 위치가 설정되지 않음")
            
        self.get_logger().info(f"=== {self.medicine_name} 약통 다시 집기 시작 ===")
        self.grip_bottle_until_success()
        
        # [참고] lock 해제를 위해 부호 확인 필요 (-17 / +17)
        self.movel(self.posx(0, 0, 0, 0, 0, -17), vel=5, acc=5, ref=self.DR_BASE, mod=self.DR_MV_MOD_REL)
        sleep(0.5)

        self.lift_medicine_with_force()
        self.get_logger().info(f"{self.medicine_name} 약통 집기 및 힘제어 상승 완료")
        
        # 약 부으러 가기 전 기울이기 (충돌 회피 경유지)
        self.movej(self.posj(-28.01, 18.18, 29.61, -2.29, 132.72, -149.21), vel=10, acc=10)
        self.movej(self.posj(-28.01, 18.18, 29.61, -2.29, 70.82, -181.30), vel=10, acc=10)

    # [통합/추가된 로직] pour_pills_rotate.py의 약 붓기 시퀀스
    def pour_pills_sequence(self):
        self.get_logger().info("=== 조제기 위치로 이동하여 약 붓기를 시작합니다 ===")
        
        self.movejx(self.X_DISPENSER_POS, vel=self.vel_2, acc=self.acc_2, sol=2)
        
        start_pose = self.get_current_pos_base()
        start_rotation = self._zyz_to_rotation_matrix(*start_pose[3:6])
        tip_offset_base = self._matvec_3x3(start_rotation, self.bottle_tip_offset_tcp)
        tip_position_base = [start_pose[i] + tip_offset_base[i] for i in range(3)]

        angles = self._make_angle_sequence(POUR_TOTAL_ANGLE_DEG_2, POUR_STEP_DEG_2)
        reference_abc = start_pose[3:6]

        for angle_deg in angles:
            target_pose, _, reference_abc = self._calculate_virtual_tcp_pose(
                start_rotation, tip_position_base, angle_deg, reference_abc)
            self.movel(target_pose, vel=POUR_VEL_2, acc=POUR_ACC_2, ref=self.DR_BASE)
            
        sleep(POUR_HOLD_SEC_2)
        self.get_logger().info("약 붓기 동작 완료")
        
        # 원상복구
        self.movejx(self.X_DISPENSER_POS, vel=self.vel_2, acc=self.acc_2, sol=2)
        self.movej(self.posj(-28.01, 18.18, 29.61, -2.29, 70.82, -181.30), vel=10, acc=10)
        self.movej(self.posj(-28.01, 18.18, 29.61, -2.29, 132.72, -149.21), vel=10, acc=10)

    def open_drawer(self):
        if self.X_DRAWER is None:
            raise RuntimeError("서랍 열기 위치가 설정되지 않음")

        self.get_logger().info("서랍 열기 시작")

        self.movej(
            self.posj(*DRAWER_SAFE_JOINT_2),
            vel=20,
            acc=20,
        )

        self.movej(
            self.X_DRAWER,
            vel=self.vel,
            acc=self.acc,
            ref=self.DR_BASE
        )

        self.grip()

        self.movel(
            self.posx(-125, 0, 0, 0, 0, 0),
            vel=20,
            acc=20,
            ref=self.DR_BASE,
            mod=self.DR_MV_MOD_REL,
        )

        self.release()

        x, y, z, rx, ry, rz = self.get_current_pos_base()
        self.X_DRAWER_CLOSED = self.posx(x, y, z, rx, ry, rz)

        self.get_logger().info(
            "서랍 열기 완료: "
            f"x={x:.2f}, y={y:.2f}, z={z:.2f}, "
            f"rx={rx:.2f}, ry={ry:.2f}, rz={rz:.2f}"
        )

        # 열린 서랍에서 약 보관 위치로 이동하기 전에 안전 Joint로 복귀한다.
        self.movej(
            self.posj(*DRAWER_SAFE_JOINT_2),
            vel=20,
            acc=20,
        ) 

    def close_drawer(self):
        if self.X_DRAWER_CLOSED is None:
            raise RuntimeError("서랍을 연 후 위치가 저장되지 않음")

        if self.X_DRAWER is None:
            raise RuntimeError("서랍 위치가 설정되지 않음")

        self.get_logger().info("서랍 닫기 시작")

        self.movejx(
            self.X_DRAWER_CLOSED,
            vel=self.vel,
            acc=self.acc,
            ref=self.DR_BASE,
            sol=2,
        )

        self.movel(
            self.posx(125, 0, 0, 0, 0, 0),
            vel=10,
            acc=10,
            ref=self.DR_BASE,
            mod=self.DR_MV_MOD_REL,
        )

        self.get_logger().info("서랍 닫기 완료")
    
    def notify_done(self):
        if self.medicine_name is None:
            raise RuntimeError("medicine_name이 설정되지 않음")

        if self.refill_amount is None:
            raise RuntimeError("refill_amount가 설정되지 않음")

        payload = {
            "medicine_name": self.medicine_name,
            "amount": self.refill_amount,
        }

        self.get_logger().info(f"리필 완료 POST 전송: {payload}")

        try:
            response = requests.post(
                self.DONE_URL,
                json=payload,
                timeout=3,
            )

            if response.ok:
                try:
                    response_data = response.json()
                except ValueError:
                    response_data = response.text

                self.get_logger().info(
                    f"리필 완료 POST 성공: "
                    f"{response_data}"
                )

            else:
                self.get_logger().error(
                    f"리필 완료 POST 실패: "
                    f"status={response.status_code}, "
                    f"body={response.text}"
                )

        except requests.RequestException as e:
            self.get_logger().error(
                f"리필 완료 POST 통신 오류: {e}"
            )

    # --------------------------------------------------
    # 통합된 전체 작업 실행
    # --------------------------------------------------
    def run(self):
        test_task = TEST_TASKS[TEST_MEDICINE_NUMBER]
        self.get_logger().info("=== 전체 작업 시작 (Opener + Pourer 연속 실행) ===")
        self.open_drawer()
        
        # 1단계: Opener 파트 (뚜껑 열고 버리기)
        success = False
        if self.lid_type == "spin":
            self.storage_grasp()
            if self.fix_lid():
                if self.spin_lid():
                    self.trash()
                    success = True
        elif self.lid_type == 'pull': 
            self.storage_grasp()
            if self.fix_lid():
                if self.pull_lid():
                    self.trash()
                    success = True
        else: 
            self.get_logger().error(f"알 수 없는 뚜껑 타입입니다: {self.lid_type}")
            return
            
        # 2단계: Pourer 파트 (약통 집고 조제기로 가서 약 붓기)
        if success:
            self.get_logger().info("=== [전환] 약 붓기 시퀀스를 이어서 시작합니다 ===")
            try:
                self.pick_medicine() 
                self.pour_pills_sequence() 
            except Exception as e:
                self.get_logger().error(f"약 붓기 동작 중 예외 발생: {e}")
        else:
            self.get_logger().error("뚜껑 열기 단계 실패로 인해 약 붓기 시퀀스를 진행하지 않습니다.")
            
        # 3단계: 작업 완료 및 종료 (최초 Ready 위치로 원복)
        self.movej(self.J_READY, vel=self.vel_1, acc=self.acc_1)
        self.get_logger().info("=== 전체 작업 완료 및 대기 위치 복귀 ===")

def main(args=None):
    rclpy.init(args=args)
    robot = PharmacyRobot()
    try:
        rclpy.spin(robot)
    except KeyboardInterrupt:
        robot.get_logger().info("Keyboard Interrupt")
    except Exception as e:
        robot.get_logger().error(f"Robot error: {e}")
    finally:
        robot.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
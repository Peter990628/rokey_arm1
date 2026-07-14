import rclpy
import DR_init
from time import sleep
import threading
from rclpy.node import Node
from std_msgs.msg import String
import json
import math


ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60


DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

POUR_TOTAL_ANGLE_DEG = -30.0
POUR_STEP_DEG = 6
POUR_VEL = 5
POUR_ACC = 5
POUR_HOLD_SEC = 1.0


ON, OFF = 1, 0


class Opener(Node):
    def __init__(self):
        super().__init__('opener')

        DR_init.__dsr__node = self

        from DSR_ROBOT2 import (
            set_digital_output,
            get_digital_input,
            set_tool,
            set_tcp,
            movej,
            movel,
            movejx,
            wait,
            trans,
            task_compliance_ctrl,
            get_tool_force,
            amove_periodic,
            check_position_condition,
            set_desired_force,
            get_current_posx,
            release_force,
            task_compliance_ctrl,
            release_compliance_ctrl,
            amovel,
            get_tool_force,
            DR_TOOL,
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
        self.movejx = movejx  # 누락되었던 movejx 등록
        self.wait = wait
        self.trans = trans
        self.task_compliance_ctrl = task_compliance_ctrl
        self.get_tool_force = get_tool_force
        self.amove_periodic = amove_periodic
        self.check_position_condition = check_position_condition
        self.set_desired_force = set_desired_force
        self.get_current_posx = get_current_posx
        self.release_force = release_force
        self.amovel = amovel              
        self.get_tool_force = get_tool_force 

        # DSR 상수 저장
        self.DR_BASE = DR_BASE
        self.DR_MV_MOD_ABS = DR_MV_MOD_ABS
        self.DR_MV_MOD_REL = DR_MV_MOD_REL
        self.DR_TOOL = DR_TOOL
        self.posj = posj
        self.posx = posx
        self.vel = VELOCITY
        self.acc = ACC
        self.DR_AXIS_Z = DR_AXIS_Z
        self.DR_SSTOP = DR_SSTOP

        self.X_LOCK_RETURN = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.bottle_tip_offset_tcp = [0.0, 0.0, 75]

        self.define_positions()

        # 위치 생성 함수
        self.posj = posj
        self.posx = posx

        # 기본 속도 / 가속도
        self.vel = VELOCITY
        self.acc = ACC

        # 초기 변수 선언
        self.X_LOCK_RETURN = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.lid_type = None

        # Subscriber
        self.create_subscription(String, "/dsr01/pharmacy/medicine", self.storage_location_callback, 10)

        # 자주 쓰는 위치 정의
        self.define_positions()

        # 로봇 초기 세팅
        self.init_robot()

    # --------------------------------------------------
    # 0. 위치 정의
    # --------------------------------------------------

    def define_positions(self):

        """

        posj: 관절 좌표
        posx: 직교 좌표 x, y, z, rx, ry, rz

        """
        # 대기 위치
        self.J_READY = self.posj(0, 0, 90, 0, 90, 0)

        # 적재소 근처 위치
        self.X_STORAGE_APPROACH = self.posx(357.12, 219.79, 200.52, 96.00, 176.96, 105.65)

        # 거치대에 약통 꽂을 위치
        self.X_FIX_ABOVE = self.posx(550.90, 1.02, 200.52, 8.33, -179.62, 18.10)
        self.X_FIX = self.posx(551.90, 2.2, 50.91, 8.33, -179.62, 18.10)

        # 거치대에 약통 꽂을 위치의 살짝 위 (돌려서 여는 뚜껑)
        self.X_SPIN_LID_ABOVE = self.posx(551.90, 2.2, 100.91, 8.33, -179.62, 18.10)

        # 병따개 거치 위치
        self.X_OPENER_TOOL_ABOVE = self.posx(582.33, 244.77, 129.81, 39.98, 180.00, 132.74)
        self.X_OPENER_TOOL = self.posx(582.33, 244.77, 99.81, 39.98, 180.00, 132.74)

        # 뚜껑 열기 위해 병따개 걸 위치
        self.X_OPEN_READY_ABOVE = self.posx(602.07, 6.05, 182.47, 161.74, -179.88, 157.58)        
        self.X_OPEN_READY = self.posx(602.07, 6.05, 148.47, 161.74, -179.88, 157.58)        # x축 위치로 조금 가게 설정
        self.X_OPEN_LOC = self.posx(557.46, 5.58, 146.22, 104.54, -179.97, 100.42)

        # 쓰레기통 위치 (뚜껑 버리는)
        self.X_TRASH = self.posx(-423.12, -96.12, 89.41, 8.51, -179.18, 101.63)

    # --------------------------------------------------
    # 0. 기타 함수 정의
    # --------------------------------------------------

    def get_logger(self):
        return self.node.get_logger()
    
    def get_current_pos_base(self):
        current_result = self.get_current_posx(ref=self.DR_BASE)
        if current_result is None:
            raise RuntimeError("현재 위치 조회 결과가 None임")

        if (
            hasattr(current_result, "__len__")
            and len(current_result) >= 6
            and isinstance(current_result[0], (int, float))
        ):
            current_pos = current_result
        elif (
            hasattr(current_result, "__len__")
            and len(current_result) >= 1
        ):
            current_pos = current_result[0]
        else:
            current_pos = None

        if current_pos is None or len(current_pos) < 6:
            raise RuntimeError(
                f"현재 위치 데이터가 올바르지 않음: {current_result}"
            )

        return [float(value) for value in current_pos[:6]]
    
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

    def _calculate_virtual_tcp_pose(
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
        target_abc = self._rotation_matrix_to_zyz(
            target_rotation,
            reference_abc,
        )
        target_values = target_position + target_abc
        target_pose = self.posx(*target_values)

        return target_pose, target_values, target_abc

    # --------------------------------------------------
    # 0. 그리퍼 열기
    # --------------------------------------------------
    def release(self):

        self.node.get_logger().info("그리퍼 열기")

        self.set_digital_output(1, OFF)
        self.set_digital_output(2, OFF)

        self.set_digital_output(1, ON)
        self.set_digital_output(2, OFF)


        sleep(1)


    # --------------------------------------------------
    # 0. 그리퍼 닫기
    # --------------------------------------------------
    def grip(self):

        self.node.get_logger().info("그리퍼 닫기")

        self.set_digital_output(1, OFF)
        self.set_digital_output(2, OFF)

        self.set_digital_output(1, OFF)
        self.set_digital_output(2, ON)


        sleep(1)

    # --------------------------------------------------
    # 0. 그리퍼 살짝 닫기
    # --------------------------------------------------
    def little_grip(self):

        self.node.get_logger().info("그리퍼 살짝 닫기")

        self.set_digital_output(1, OFF)
        self.set_digital_output(2, OFF)

        self.set_digital_output(1, ON)
        self.set_digital_output(2, ON)


        sleep(1)

    
    # --------------------------------------------------
    # 1. 로봇 초기 세팅
    # --------------------------------------------------
    def init_robot(self):

        self.node.get_logger().info("로봇 초기 세팅 및 레디 포즈 이동")

        # 실제 등록된 tool/tcp 이름으로 수정 필요
        self.set_tool("Tool Weight_1")
        self.set_tcp("Tool_v1")

        self.release()
        self.movej(self.J_READY, vel=self.vel, acc=self.acc)
        self.wait(1.0)


    # --------------------------------------------------
    # 2. Subscribe 성공
    # --------------------------------------------------
    def storage_location_callback(self, msg):
        try:
            data = json.loads(msg.data)
            self.medicine_name = data['medicine_name']
            self.storage_loc_x = data['storage_x']
            self.storage_loc_y = data['storage_y']
            self.storage_loc_z = data['storage_z']
            self.storage_loc_dx = data['dispensing_x']
            self.storage_loc_dy = data['dispensing_y']
            self.storage_loc_dz = data['dispensing_z']
            self.lid_type = data['lid_type']
            self.storage_loc = self.posx(self.storage_loc_x, self.storage_loc_y, self.storage_loc_z, 
                                     self.storage_loc_dx, self.storage_loc_dy, self.storage_loc_dz)
            self.get_logger().info(f"수신 성공! : {self.medicine_name}의 리필을 시작합니다. (뚜껑 타입: {self.lid_type})")
            self.run()

            # threading.Thread(target=self.run, daemon=True).start()

        except Exception as e:
            self.get_logger().error(f"Callback JSON Parsing Error: {e}")


    # --------------------------------------------------
    # 적재소에서 약통 잡기
    # --------------------------------------------------

    def storage_grasp(self):

        """
  
        1. 약통의 위치로 이동 (TCP)
        2. 그리퍼 닫기 (약통 잡기)
        3. 들어올리기

        """

        self.node.get_logger().info("=== 적재소 약통 파지 시퀀스 ===")
        
        # 약통의 위치 근처로 이동 (반드시 위에서 잡아야함)
        self.node.get_logger().info("1. 적재소 근처로 이동")
        self.movejx(
            self.X_STORAGE_APPROACH, vel=self.vel, acc=self.acc, ref=self.DR_BASE, sol=2)

        # 약통의 위치로 이동 (반드시 위에서 잡아야함, 약통보다 살짝 위임)
        self.node.get_logger().info("2. 약통 위치로 이동")
        self.movejx(
            self.storage_loc, vel=self.vel, acc=self.acc, ref=self.DR_BASE, sol=2
        )

        # 약 잡을 수 있는 위치로 이동: 현재 위치 기준 z 방향 상대 이동 (아래로 27mm 정도)
        self.node.get_logger().info("3. 툴 삽입을 위한 미세 하강")
        self.movel(
            self.posx(0, 0, -27, 0, 0, 0), vel=20, acc=20, ref=self.DR_BASE, mod=self.DR_MV_MOD_REL
        )

        # 그리퍼 닫기
        self.grip()

        # 들어올리기: 현재 위치 기준 z 방향 상대 이동 (위로 40mm 정도)
        self.node.get_logger().info("4. 약통 파지 후 안전 고도로 이동")
        self.movel(
            self.posx(0, 0, 40, 0, 0, 0), vel=20, acc=20, ref=self.DR_BASE, mod=self.DR_MV_MOD_REL
        )
        self.wait(0.5)
        self.movejx(self.X_STORAGE_APPROACH, vel=self.vel, acc=self.acc, ref=self.DR_BASE, sol=2)
        self.wait(0.5)


    # --------------------------------------------------
    # 거치대에 꽂기
    # --------------------------------------------------
    def pull_down(self):

        """

        거치대에 약통이 완전히 닿아 고정될 때까지 힘 제어로 내림

        """
        self.node.get_logger().info("거치대에 약통 꽂기 시도")
        
        self.task_compliance_ctrl(stx=[500, 500, 100, 5000, 5000, 5000])
        self.wait(0.5)
        
        self.set_desired_force(fd=[0, 0, -35, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0])
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

    # --------------------------------------------------
    # 반시계 방향으로 회전시켜 고정하기
    # --------------------------------------------------
    def lock(self):

        self.node.get_logger().info("반시계 방향 회전을 통해 거치대 고정 시도 (Lock)")    

        self.movej(self.posj(0, 0, 0, 0, 0, -17), vel=3, acc=1, mod=self.DR_MV_MOD_REL)
        self.wait(0.5)

    # --------------------------------------------------
    # 약통 고정
    # --------------------------------------------------

    def fix_lid(self):
        self.node.get_logger().info("=== 약통 거치대 고정 시퀀스 ===")     
        
        self.node.get_logger().info("1. 약통 거치대 force 시작 위치의 상공으로 이동")
        self.movejx(self.X_FIX_ABOVE, vel=30, acc=30, ref=self.DR_BASE, sol=2)
        self.wait(1)

        self.node.get_logger().info("2. 약통 거치대 force 시작 위치로 이동")
        self.movejx(self.X_FIX, vel=5, acc=5, ref=self.DR_BASE, sol=2)
        self.wait(0.5)

        self.node.get_logger().info("3. 거치대 소켓에 가압 삽입 (pull_down)")
        self.pull_down()
        self.wait(0.5)

        self.node.get_logger().info("4. 반시계 방향 회전하여 락킹 (lock)")
        self.lock()
        self.wait(0.5)

        # 현재 위치 실측 저장 
        self.X_LOCK_RETURN = self.get_current_posx(ref=self.DR_BASE)[0]
        self.node.get_logger().info(f"동적 좌표 저장 완료: {self.X_LOCK_RETURN}")

        self.node.get_logger().info("4. 그리퍼 해제 후 수직 안전 탈출")
        self.release()

        self.movel(self.posx(0, 0, 100, 0, 0, 0), vel=20, acc=20, ref=self.DR_BASE, mod=self.DR_MV_MOD_REL)

    # --------------------------------------------------
    # 당겨서 뚜껑 열기 필요한 함수 정의
    # --------------------------------------------------

    # (2) 뚜껑에 병따개 가까이 가져가기
    def pull_lid_open(self):
        self.node.get_logger().info("X_OPEN_READY_ABOVE 위치로 이동")
        self.movejx(self.X_OPEN_READY_ABOVE,
                vel=10,
                acc=10,
                ref=self.DR_BASE,
                sol=2)
        self.wait(0.5)

        self.movel(self.X_OPEN_READY,
                vel=5,
                acc=5,
                ref=self.DR_BASE)
        self.wait(0.5)

        found_contact = False

        self.node.get_logger().info("X축 방향 탐색 시작")

        for _ in range(100):

            current_force = self.get_tool_force(ref=self.DR_BASE)
            self.node.get_logger().info(f"현재 force ({current_force[0]:.2f}N)")

            if current_force[0] > 0:
                found_contact = True
                self.node.get_logger().info(
                    f"X축 충돌 감지 ({current_force[0]:.2f}N)"
                )
                break

            self.movel(
                self.posx(-2.0, 0, 0, 0, 0, 0),
                vel=5,
                acc=5,
                ref=self.DR_BASE,
                mod=self.DR_MV_MOD_REL,
            )

            sleep(0.05)

        if not found_contact:
            self.node.get_logger().error("뚜껑을 찾지 못했습니다.")

    
    # (3) 뚜껑에 병따개 달기
    def pour_tweezer(self):
        self.grip()
        """활성 TCP를 바꾸지 않고 DB의 약병 끝점 오프셋을 중심으로 붓는다."""
        if self.bottle_tip_offset_tcp is None:
            raise RuntimeError("약병 끝점 오프셋이 설정되지 않음")

        start_pose = self.get_current_pos_base()
        start_position = start_pose[:3]
        start_abc = start_pose[3:6]

        start_rotation = self._zyz_to_rotation_matrix(*start_abc)
        tip_offset_base = self._matvec_3x3(
            start_rotation,
            self.bottle_tip_offset_tcp,
        )

        tip_position_base = [
            start_position[i] + tip_offset_base[i]
            for i in range(3)
        ]

        self.get_logger().info(
            "약 붓기 시작: "
            f"start={[round(v, 3) for v in start_pose]}, "
            f"offset={self.bottle_tip_offset_tcp}, "
            f"tip_base={[round(v, 3) for v in tip_position_base]}"
        )

        angles = self._make_angle_sequence(
            POUR_TOTAL_ANGLE_DEG,
            POUR_STEP_DEG,
        )

        calculated_poses = []
        reference_abc = start_abc

        for angle_deg in angles:
            target_pose, target_values, reference_abc = (
                self._calculate_virtual_tcp_pose(
                    tip_position_base=tip_position_base,
                    start_rotation=start_rotation,
                    angle_deg=angle_deg,
                    reference_abc=reference_abc,
                )
            )
            calculated_poses.append(target_pose)

            self.get_logger().info(
                f"붓기 {angle_deg:.1f}도 목표: "
                f"{[round(v, 3) for v in target_values]}"
            )

            self.movel(
                target_pose,
                vel=POUR_VEL,
                acc=POUR_ACC,
                ref=self.DR_BASE,
            )
        sleep(POUR_HOLD_SEC)
        self.get_logger().info("병따개 뚜껑에 걸기 완료")
    
    
    # --------------------------------------------------
    # 당겨서 뚜껑 열기 시퀀스 
    # --------------------------------------------------

    def pull_lid(self):
        self.node.get_logger().info("=== 당겨서 여는 약통 뚜껑 열기 ===")

        # (1)-1 병따개 거치대로 이동
        self.movejx(
            self.X_OPENER_TOOL_ABOVE, vel=30, acc=30, ref=self.DR_BASE, sol=2)
        self.movel(
            self.posx(0, 0, -30, 0, 0, 0), vel=30, acc=30, ref=self.DR_BASE, mod=self.DR_MV_MOD_REL
        )
        
        self.wait(0.5)

        # (1)-2 병따개 그립
        self.grip()
        
        # (1)-3 병따개 위로 꺼내기
        self.movel(
            self.posx(0, 0, 130, 0, 0, 0), vel=30, acc=30, ref=self.DR_BASE, mod=self.DR_MV_MOD_REL
        )
        self.wait(0.5)

        # 나중에 돌아올 병따개 거치 위치 저장
        self.X_OPENER_RETURN = self.get_current_posx(ref=self.DR_BASE)[0]

        # (2) 당겨서 뚜껑 열기
        self.pull_lid_open()

        # (3) 뚜껑에 병따개 달기
        self.pour_tweezer()

        # (4) 병따개로 뚜껑 따기
        self.movel(self.posx(0, 0, -150, 0, 0, 0), vel=60, acc=60, ref=self.DR_TOOL, mod=self.DR_MV_MOD_REL)

        # (5) 병따개 제자리에 가져다 놓기
        self.movejx(self.X_OPENER_RETURN, vel=self.vel, acc=self.acc, ref=self.DR_BASE, sol=0)
        self.movel(self.posx(0, 0, -100, 0, 0, 0), vel=15, acc=15, ref=self.DR_BASE, mod=self.DR_MV_MOD_REL)
        
        self.wait(0.5)

        self.release()

    # --------------------------------------------------
    # 돌려서 뚜껑 열기 필요한 함수 정의
    # --------------------------------------------------

    def spin_open(self):
        self.node.get_logger().info("라쳇 모션 최적화 시작 (Z축 보정 적용)")
        
        # 1. 순응 제어 및 가압 세팅
        self.task_compliance_ctrl([10000, 10000, 300, 10000, 10000, 10000])
        self.set_desired_force(fd=[0, 0, -20, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0])
        self.wait(0.5)

        # 2. 초기 뚜껑 풀림 펄스
        for _ in range(3):
            self.movel(self.posx(0, 0, 0, 0, 0, -10), vel=50, acc=50, ref=self.DR_TOOL, mod=self.DR_MV_MOD_REL)
            self.wait(0.1) 
            
        # 3. 뚜껑 열기 판단 루프 (최대 5회 시도)
        max_attempts = 5
        for attempt in range(max_attempts):
            self.node.get_logger().info(f"[{attempt + 1}/{max_attempts}] 뚜껑 회전 시도 중...")
            
            # (1) 반시계 방향으로 90도씩 두 번 회전 (총 반시계 방향으로 180도 회전)
            for _ in range(2):
                self.movel(self.posx(0, 0, 0, 0, 0, -90), vel=25, acc=25, ref=self.DR_TOOL, mod=self.DR_MV_MOD_REL)
            
            self.release()
            self.wait(0.3)
            
            # (2) 원위치 복귀 (그리퍼는 열려있으므로 Z축 보정값 포함하여 복귀)
            for _ in range(2):
                self.movel(self.posx(0, 0, 0, 0, 0, 90), vel=30, acc=30, ref=self.DR_TOOL, mod=self.DR_MV_MOD_REL)
                self.movel(self.posx(0, 0, -1, 0, 0, 0), vel=30, acc=30, ref=self.DR_TOOL, mod=self.DR_MV_MOD_REL)
        
            # (3) 뚜껑을 다시 잡기 위해 더 밀착 
            self.movel(self.posx(0, 0, 3, 0, 0, 0), vel=25, acc=25, ref=self.DR_TOOL, mod=self.DR_MV_MOD_REL)
            self.grip()
            self.wait(0.5)
            
            # (4) 뚜껑이 완전히 열렸는지 당겨서 확인을 위해 누르는 힘 해제 
            self.release_force()
            self.release_compliance_ctrl()
            self.wait(0.5)

            # (5) 위로 살짝 올려보며 물리적 저항 확인
            is_opened = True
            moved_z = 0
            self.node.get_logger().info("위로 당겨서 뚜껑 분리 상태를 확인합니다.")
            for step in range(1):  # 2mm씩 최대 5번 (총 10mm) 조심스럽게 당겨봅니다
                self.node.get_logger().info(f"{step+1}번째 확인")
                # 툴 기준 -Z (위쪽) 방향으로 이동
                self.movel(self.posx(0, 0, -1, 0, 0, 0), vel=15, acc=15, ref=self.DR_TOOL, mod=self.DR_MV_MOD_REL)
                moved_z += 1
                
                # 들어올릴 때 저항력(Force) 측정
                current_force = self.get_tool_force(ref=self.DR_TOOL)
                
                # Z축(툴 기준) 당겨지는 힘이 임계값(예: 15N) 이상 걸리면 나사산에 아직 물려있는 것
                if abs(current_force[2]) > 3.0:
                    self.node.get_logger().info(f"뚜껑 저항 감지 ({abs(current_force[2]):.1f}N). 아직 열리지 않았습니다.")
                    is_opened = False
                    break
            
            if is_opened:
                # 힘이 걸리지 않고 무사히 당겨졌다면 뚜껑 분리 성공!
                self.node.get_logger().info("뚜껑 분리에 성공했습니다! 완전히 들어올립니다.")
                # 마저 위로 쭉 들어올리기
                self.movel(self.posx(0, 0, -40, 0, 0, 0), vel=20, acc=20, ref=self.DR_TOOL, mod=self.DR_MV_MOD_REL)
                break  # 성공했으므로 반복 루프 탈출
            else:
                self.release()
                # 안 열렸다면, 위로 당겨본 만큼(moved_z) 다시 내려가서 원위치
                self.movel(self.posx(0, 0, moved_z, 0, 0, 0), vel=15, acc=15, ref=self.DR_TOOL, mod=self.DR_MV_MOD_REL)
                self.grip()

                # 다음 회전을 위해 가압(누르기) 제어 다시 가동
                self.node.get_logger().info("다시 누르면서(가압) 다음 회전을 준비합니다.")
                self.task_compliance_ctrl([10000, 10000, 300, 10000, 10000, 10000])
                self.set_desired_force(fd=[0, 0, -20, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0])
                self.wait(0.5)

                self.movel(self.posx(0, 0, 5, 0, 0, 0), vel=15, acc=15, ref=self.DR_TOOL, mod=self.DR_MV_MOD_REL)
                self.wait(0.5)
        else:
            self.node.get_logger().warn(f"최대 시도 횟수({max_attempts}회)를 초과했습니다. 안전을 위해 공정을 중단합니다.")
            
        self.node.get_logger().info("약통 뚜껑 개폐 시퀀스 완료")

    # 뚜껑 버리는 함수
    def trash(self):
        self.movejx(
            self.X_TRASH, vel=self.vel, acc=self.acc, ref=self.DR_BASE, sol=2
        )

        # 그리퍼 열기
        self.release()

        self.node.get_logger().info("약통 뚜껑 버리기 완료")

    # --------------------------------------------------
    # 돌려서 뚜껑 열기 시퀀스 
    # --------------------------------------------------
        """

        1. 약통 꽂힌 위치의 위로 이동
        2. 그리퍼 닫기
        3. 반시계 방향으로 회전시켜 열기
        4. 뚜껑 버리기
        
        """

    def spin_lid(self):
        self.node.get_logger().info("=== 돌려서 약통 뚜껑 열기 ===")

        # (1) 약통 꽂힌 위치의 위로 이동
        self.movejx(
            self.X_SPIN_LID_ABOVE, vel=self.vel, acc=self.acc, ref=self.DR_BASE, sol=2
        )

        # (2) 약통 뚜껑 파지를 위해 위치 조정
        self.movel(
            self.posx(0, 0, -35, 0, 0, 0), 
            vel=15, acc=15, 
            ref=self.DR_BASE, 
            mod=self.DR_MV_MOD_REL
        )

        # (3) 그리퍼 닫기
        self.grip()

        # (4) 뚜껑 열기
        self.spin_open()

        # (5) 뚜껑 버리기
        self.trash()

    # --------------------------------------------------
    # 뚜껑 버리기
    # --------------------------------------------------
    def trash(self):
        self.movejx(
            self.X_TRASH, vel=self.vel, acc=self.acc, ref=self.DR_BASE, sol=2
        )

        # 그리퍼 열기
        self.release()

        self.node.get_logger().info("약통 뚜껑 버리기 완료")


    # --------------------------------------------------
    # 전제 작업 실행
    # --------------------------------------------------
    def run(self):

        self.node.get_logger().info("전체 작업 시작")

        if self.lid_type == "spin":

            self.storage_grasp()
            self.fix_lid()
            self.spin_lid()
            self.trash()

        if self.lid_type == 'pull':

            self.storage_grasp()
            self.fix_lid()
            self.pull_lid()

        else: 
            self.node.get_logger().info("값이 없습니다")

        self.node.get_logger().info("전체 작업 완료")

  
def main(args=None):
    rclpy.init(args=args)

    robot = Opener()

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
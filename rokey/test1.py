# 약품 리필 동작 전체 흐름을 실제 로봇으로 시험하는 통합 테스트 코드
# by syc

import math
from time import sleep

import rclpy
import DR_init


ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

TCP_NAME = "Tool_v1"
TOOL_NAME = "Tool Weight_1"

VELOCITY = 50
ACC = 50

ON = 1
OFF = 0

# 1=탁센, 2=타이레놀, 3=별사탕
TEST_MEDICINE_NUMBER = 3

# 힘제어로 약통을 DR_BASE +Z 방향으로 들어 올릴 목표 거리 [mm]
# pour_pills_rotate의 go_to_tool()과 동일한 값
MEDICINE_LIFT_DISTANCE = 40.0

# 순응제어/힘제어 설정
COMPLIANCE_STX = [10000, 10000, 700, 300, 300, 300]
DESIRED_FORCE = [0, 0, 40, 0, 0, 0]
FORCE_DIRECTION = [0, 0, 1, 0, 0, 0]
FORCE_CHECK_INTERVAL_SEC = 0.05

# 약 붓기 설정: 현재 TCP의 로컬 X축을 중심으로 회전
POUR_TOTAL_ANGLE_DEG = 60.0
POUR_STEP_DEG = 10.0
POUR_VEL = 15
POUR_ACC = 15
POUR_HOLD_SEC = 1.0

# 실제 토픽으로 받을 데이터와 같은 필드명을 그대로 사용
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
        "bottle_tip_offset_y": 0.0,
        "bottle_tip_offset_z": 0.0,
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
        "bottle_tip_offset_y": 0.0,
        "bottle_tip_offset_z": 0.0,
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


# 반드시 DSR_ROBOT2 import 전에 설정
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


class PourPills:
    def __init__(
        self,
        node,
        dsr_functions,
        dsr_constants,
        posx,
        posj,
    ):
        self.node = node
        self.wait = dsr_functions["wait"]
        self.set_digital_output = dsr_functions["set_digital_output"]
        self.set_tool = dsr_functions["set_tool"]
        self.set_tcp = dsr_functions["set_tcp"]
        self.movel = dsr_functions["movel"]
        self.movej = dsr_functions["movej"]
        self.movejx = dsr_functions["movejx"]
        self.task_compliance_ctrl = dsr_functions["task_compliance_ctrl"]
        self.set_desired_force = dsr_functions["set_desired_force"]
        self.get_current_posx = dsr_functions["get_current_posx"]
        self.release_force = dsr_functions["release_force"]
        self.release_compliance_ctrl = dsr_functions["release_compliance_ctrl"]

        self.DR_BASE = dsr_constants["DR_BASE"]
        self.DR_MV_MOD_REL = dsr_constants["DR_MV_MOD_REL"]
        self.DR_FC_MOD_REL = dsr_constants["DR_FC_MOD_REL"]  

        self.posx = posx
        self.posj = posj

        self.vel = VELOCITY
        self.acc = ACC

        self.current_task = None
        self.medicine_id = None
        self.medicine_name = None

        # 약 보관 위치: BASE 기준 posx
        self.storage_x = None
        self.storage_y = None
        self.storage_z = None
        self.storage_rx = None
        self.storage_ry = None
        self.storage_rz = None

        # 조제기(약 붓기 시작 위치): Joint J1~J6
        self.dispensing_x = None
        self.dispensing_y = None
        self.dispensing_z = None
        self.dispensing_rx = None
        self.dispensing_ry = None
        self.dispensing_rz = None

        # 활성 TCP에서 약병 끝점까지의 로컬 오프셋 [mm]
        self.bottle_tip_offset_tcp = None

        # 서랍 접근 위치: Joint J1~J6
        self.drawer_x = None
        self.drawer_y = None
        self.drawer_z = None
        self.drawer_rx = None
        self.drawer_ry = None
        self.drawer_rz = None

        self.X_STORAGE = None
        self.X_DISPENSER_POS = None
        self.X_DRAWER = None

        # 전체 코드에서 사용하는 쓰레기통 BASE 기준 TCP 위치
        self.X_TRASH_DROP = self.posx(
            -423.12,
            -96.72,
            89.41,
            8.51,
            -179.18,
            101.63,
        )

        # 실제 의미는 서랍을 연 직후의 TCP 위치
        self.X_DRAWER_CLOSED = None

        # 전체 코드와 동일하게 객체 생성 시 로봇 초기화
        self.init_robot()

    def get_logger(self):
        return self.node.get_logger()

    # --------------------------------------------------
    # 토픽 대신 테스트 딕셔너리를 넣지만,
    # 실제 전체 코드와 같은 방식으로 현재 작업과 위치를 설정
    # --------------------------------------------------
    def set_task_from_data(self, task):
        required_keys = [
            "medicine_number",
            "medicine_name",

            "storage_x",
            "storage_y",
            "storage_z",
            "storage_rx",
            "storage_ry",
            "storage_rz",

            "dispensing_x",
            "dispensing_y",
            "dispensing_z",
            "dispensing_rx",
            "dispensing_ry",
            "dispensing_rz",

            "bottle_tip_offset_x",
            "bottle_tip_offset_y",
            "bottle_tip_offset_z",

            "drawer_x",
            "drawer_y",
            "drawer_z",
            "drawer_rx",
            "drawer_ry",
            "drawer_rz",
        ]

        for key in required_keys:
            if key not in task:
                raise KeyError(f"필수 데이터가 없음: {key}")

        self.current_task = task
        self.medicine_id = int(task["medicine_number"])
        self.medicine_name = str(task["medicine_name"])

        # storage_*는 BASE 기준 TCP 좌표이므로 posx
        self.storage_x = float(task["storage_x"])
        self.storage_y = float(task["storage_y"])
        self.storage_z = float(task["storage_z"])
        self.storage_rx = float(task["storage_rx"])
        self.storage_ry = float(task["storage_ry"])
        self.storage_rz = float(task["storage_rz"])

        self.X_STORAGE = self.posx(
            self.storage_x,
            self.storage_y,
            self.storage_z,
            self.storage_rx,
            self.storage_ry,
            self.storage_rz,
        )

        # dispensing_*는 Joint J1~J6 값이므로 posj
        self.dispensing_x = float(task["dispensing_x"])
        self.dispensing_y = float(task["dispensing_y"])
        self.dispensing_z = float(task["dispensing_z"])
        self.dispensing_rx = float(task["dispensing_rx"])
        self.dispensing_ry = float(task["dispensing_ry"])
        self.dispensing_rz = float(task["dispensing_rz"])

        self.X_DISPENSER_POS = self.posj(
            self.dispensing_x,
            self.dispensing_y,
            self.dispensing_z,
            self.dispensing_rx,
            self.dispensing_ry,
            self.dispensing_rz,
        )

        self.bottle_tip_offset_tcp = [
            float(task["bottle_tip_offset_x"]),
            float(task["bottle_tip_offset_y"]),
            float(task["bottle_tip_offset_z"]),
        ]

        # drawer_*는 Joint J1~J6 값이므로 posj
        self.drawer_x = float(task["drawer_x"])
        self.drawer_y = float(task["drawer_y"])
        self.drawer_z = float(task["drawer_z"])
        self.drawer_rx = float(task["drawer_rx"])
        self.drawer_ry = float(task["drawer_ry"])
        self.drawer_rz = float(task["drawer_rz"])

        self.X_DRAWER = self.posj(
            self.drawer_x,
            self.drawer_y,
            self.drawer_z,
            self.drawer_rx,
            self.drawer_ry,
            self.drawer_rz,
        )

        self.get_logger().info(
            f"테스트 작업 설정 완료: "
            f"medicine_number={self.medicine_id}, "
            f"medicine_name={self.medicine_name}, "
            f"storage_posx={list(self.X_STORAGE)}, "
            f"dispensing_posj={list(self.X_DISPENSER_POS)}, "
            f"bottle_tip_offset_tcp={self.bottle_tip_offset_tcp}, "
            f"drawer_posj={list(self.X_DRAWER)}"
        )

    # --------------------------------------------------
    # 전체 코드와 같은 초기 세팅
    # --------------------------------------------------
    def init_robot(self):
        self.get_logger().info("로봇 초기 세팅 시작")

        # 실제 전체 코드에서 Tool/TCP를 적용할 경우 동일하게 주석 해제
        self.set_tool(TOOL_NAME)
        self.set_tcp(TCP_NAME)

        self.release()

        self.get_logger().info("로봇 초기 세팅 완료")

    # --------------------------------------------------
    # 전체 코드와 동일한 그리퍼 함수
    # --------------------------------------------------
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

    # --------------------------------------------------
    # get_current_posx() 반환 형식 처리
    # --------------------------------------------------
    def get_current_pos_base(self):
        current_result = self.get_current_posx(
            ref=self.DR_BASE
        )

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

        return [
            float(value)
            for value in current_pos[:6]
        ]

    # --------------------------------------------------
    # 전체 코드에 넣을 서랍 열기 함수와 동일한 흐름
    # --------------------------------------------------
    def open_drawer(self):
        if self.X_DRAWER is None:
            raise RuntimeError("서랍 열기 위치가 설정되지 않음")

        self.get_logger().info("서랍 열기 시작")
        self.movej(
            self.posj(-42.63,-0.65,99.42,-1.37,81.22,7.57),
            vel=20,
            acc=20,
        
        )

        # Joint 위치로 이동
        self.movej(
            self.X_DRAWER,
            vel=self.vel,
            acc=self.acc,
        )

        self.grip()

        # 손잡이를 잡은 뒤 DR_BASE -X 방향으로 서랍 당기기
        self.movel(
            self.posx(
                -125,
                0,
                0,
                0,
                0,
                0,
            ),
            vel=20,
            acc=20,
            ref=self.DR_BASE,
            mod=self.DR_MV_MOD_REL,
        )
        self.release()

        current_pos = self.get_current_pos_base()
        x, y, z, rx, ry, rz = current_pos

        self.X_DRAWER_CLOSED = self.posx(
            x,
            y,
            z,
            rx,
            ry,
            rz,
        )

        self.get_logger().info(
            "서랍 열기 완료: "
            f"x={x:.2f}, "
            f"y={y:.2f}, "
            f"z={z:.2f}, "
            f"rx={rx:.2f}, "
            f"ry={ry:.2f}, "
            f"rz={rz:.2f}"
        )

        self.movej(
            self.posj(-42.63,-0.65,99.42,-1.37,81.22,7.57),
            vel=20,
            acc=20,
        
        )

    # --------------------------------------------------
    # 약 위치로 이동하여 약통을 잡고 들어 올림
    # 이 함수를 그대로 전체 코드에 넣어 사용할 수 있음
    # --------------------------------------------------
    def pick_medicine(self):
        if self.X_STORAGE is None:
            raise RuntimeError("약 보관 위치가 설정되지 않음")

        self.get_logger().info(
            f"{self.medicine_name} 약통 집기 시작"
        )

        # open_drawer()가 끝난 시점에는 손잡이를 잡고 있으므로 놓기
        self.release()

        # storage_*는 BASE 기준 posx이므로 movejx 사용
        self.get_logger().info(
            f"{self.medicine_name} 보관 위치로 이동"
        )

        self.movejx(
            self.posx(551.116, 1.157, 30.878, 14.359, -179.476, 7.189),
            vel=20,
            acc=20,
            ref=self.DR_BASE,
            sol=2,
        )

        sleep(0.5)

        # 약통 잡기
        self.grip()
        sleep(0.5)

        # 실제 프로젝트 코드와 동일하게 약통 자세를 먼저 조정
        self.movel(
            self.posx(0, 0, 0, 0, 0, -17),
            vel=5,
            acc=5,
            ref=self.DR_BASE,
            mod=self.DR_MV_MOD_REL,
        )
        sleep(0.5)
        # 고정 거리 movel 대신 순응제어 + 힘제어로 약통 들어 올리기
        self.lift_medicine_with_force()

        current_pos = self.get_current_pos_base()

        self.get_logger().info(
            f"{self.medicine_name} 약통 집기 및 힘제어 상승 완료: "
            f"current_pos={[round(value, 2) for value in current_pos]}"
        )

        self.movej(self.posj(-28.01, 18.18, 29.61, -2.29, 132.72, -149.21), vel=10, acc=10)

        # ---------------------------------------------약 부으러 가기 전 기울이기------------------
        self.movej(self.posj(-28.01, 18.18, 29.61, -2.29,  70.82, -181.30), vel=10, acc=10)



    # --------------------------------------------------
    # 실제 프로젝트 코드와 동일한 순응제어 + 힘제어 상승
    # --------------------------------------------------
    def lift_medicine_with_force(self):
        start_pose = self.get_current_pos_base()
        start_z = start_pose[2]

        self.get_logger().info(
            "약통 힘제어 상승 시작: "
            f"start_z={start_z:.2f} mm, "
            f"target={MEDICINE_LIFT_DISTANCE:.1f} mm, "
            f"force_z={DESIRED_FORCE[2]} N"
        )

        compliance_started = False
        force_started = False

        try:
            self.task_compliance_ctrl(stx = COMPLIANCE_STX)
            compliance_started = True
            self.wait(0.5)

            self.set_desired_force(
                fd=DESIRED_FORCE,
                dir=FORCE_DIRECTION,
                mod=self.DR_FC_MOD_REL
            )
            self.wait(0.5)
            force_started = True

            while rclpy.ok():
                current_pose = self.get_current_pos_base()
                current_z = current_pose[2]
                lifted_distance = current_z - start_z

                self.get_logger().info(
                    "힘제어 상승 중: "
                    f"current_z={current_z:.2f} mm, "
                    f"lifted={lifted_distance:.2f} mm"
                )

                if lifted_distance >= MEDICINE_LIFT_DISTANCE:
                    self.get_logger().info(
                        "목표 상승 거리 도달. 힘제어를 종료함"
                    )
                    break

                sleep(FORCE_CHECK_INTERVAL_SEC)

            if not rclpy.ok():
                raise RuntimeError(
                    "ROS2 종료로 약통 힘제어 상승이 중단됨"
                )

        finally:
            # 힘제어를 먼저 해제하고, 그 다음 순응제어를 해제
            if force_started:
                self.release_force()

            if compliance_started:
                self.release_compliance_ctrl()

        self.get_logger().info("약통 힘제어 상승 완료")

    

    # --------------------------------------------------
    # 약통을 든 상태에서 조제기(붓기 시작 Joint)로 이동
    # 실제 전체 코드의 move_pour()에서 붓기 직전까지와 같은 방식
    # --------------------------------------------------
    def move_to_dispensing_position(self):
        if self.X_DISPENSER_POS is None:
            raise RuntimeError("조제기 시작 Joint 위치가 설정되지 않음")

        self.get_logger().info(
            f"{self.medicine_name} 약통을 들고 조제기 위치로 이동 시작"
        )

        # dispensing_*는 Joint 값이므로 posj + movej 사용
        self.movej(
            self.X_DISPENSER_POS,
            vel=self.vel,
            acc=self.acc,
        )

        self.get_logger().info(
            f"{self.medicine_name} 조제기 붓기 시작 위치 도착"
        )

    # --------------------------------------------------
    # 약 붓기 계산: 약병 끝점을 고정하고 TCP 로컬 X축으로 회전
    # --------------------------------------------------
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

        target_abc = self._rotation_matrix_to_zyz(
            target_rotation,
            reference_abc,
        )

        target_values = target_position + target_abc
        target_pose = self.posx(*target_values)

        return target_pose, target_values, target_abc

    def pour_tweezer(self):
        if self.bottle_tip_offset_tcp is None:
            raise RuntimeError("약병 끝점 오프셋이 설정되지 않음")

        # 조제기 Joint 위치에 실제로 도달한 뒤 현재 TCP 자세를 시작점으로 사용
        start_pose = self.get_current_pos_base()
        start_position = start_pose[:3]
        start_abc = start_pose[3:6]

        start_rotation = self._zyz_to_rotation_matrix(*start_abc)

        tip_offset_base = self._matvec_3x3(
            start_rotation,
            self.bottle_tip_offset_tcp,
        )

        tip_position_base = [
            start_position[index] + tip_offset_base[index]
            for index in range(3)
        ]

        self.get_logger().info(
            "약 붓기 시작: "
            f"start={[round(value, 3) for value in start_pose]}, "
            f"offset={self.bottle_tip_offset_tcp}, "
            f"tip_base={[round(value, 3) for value in tip_position_base]}"
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
                    start_rotation=start_rotation,
                    tip_position_base=tip_position_base,
                    angle_deg=angle_deg,
                    reference_abc=reference_abc,
                )
            )

            calculated_poses.append(target_pose)

            self.get_logger().info(
                f"붓기 {angle_deg:.1f}도 목표: "
                f"{[round(value, 3) for value in target_values]}"
            )

            self.movel(
                target_pose,
                vel=POUR_VEL,
                acc=POUR_ACC,
                ref=self.DR_BASE,
            )

        sleep(POUR_HOLD_SEC)

        self.get_logger().info("같은 회전 경로로 시작 자세 복귀")

        for target_pose in reversed(calculated_poses[:-1]):
            self.movel(
                target_pose,
                vel=POUR_VEL,
                acc=POUR_ACC,
                ref=self.DR_BASE,
            )

        self.movel(
            self.posx(*start_pose),
            vel=POUR_VEL,
            acc=POUR_ACC,
            ref=self.DR_BASE,
        )

        self.get_logger().info("약 붓기 완료")

        self.movel(self.posx(0,-50,0,0,0,0), vel=20, acc=20, mod=self.DR_MV_MOD_REL)

    # --------------------------------------------------
    # 약 붓기 다음 단계: 빈 약통을 쓰레기통에 버림
    # 전체 코드의 move_trash()와 동일한 방식
    # --------------------------------------------------
    def move_trash(self):
        if self.X_TRASH_DROP is None:
            raise RuntimeError(
                "쓰레기통 위치가 설정되지 않음"
            )

        self.get_logger().info(
            "약통 버리기 시작"
        )

        # X_TRASH_DROP은 BASE 기준 posx이므로 movejx 사용
        self.movejx(
            self.X_TRASH_DROP,
            vel=self.vel,
            acc=self.acc,
            ref=self.DR_BASE,
            sol=2,
        )

        # 그리퍼를 열어 빈 약통 놓기
        self.release()

        self.get_logger().info(
            "약통 버리기 완료"
        )

    # --------------------------------------------------
    # 테스트 실행
    # 토픽 대신 직접 넣는 것만 다르고 실제 함수 흐름은 동일
    # --------------------------------------------------
    def run(self):
        if TEST_MEDICINE_NUMBER not in TEST_TASKS:
            raise ValueError(
                f"지원하지 않는 TEST_MEDICINE_NUMBER: "
                f"{TEST_MEDICINE_NUMBER}"
            )

        test_task = TEST_TASKS[TEST_MEDICINE_NUMBER]

        self.set_task_from_data(test_task)

        # 실제 전체 코드에 들어갈 순서와 동일
        self.open_drawer()
        self.pick_medicine()
        self.move_to_dispensing_position()
        self.pour_tweezer()
        self.move_trash()

        self.get_logger().info(
            "서랍 열기 → 약통 집기 → 상승 → "
            "조제기 이동 → 약 붓기 → 원위치 복귀 → "
            "약통 버리기 테스트 완료"
        )


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = rclpy.create_node(
            "pour_and_trash_test",
            namespace=ROBOT_ID,
        )

        DR_init.__dsr__node = node

        from DSR_ROBOT2 import (
            set_digital_output,
            set_tool,
            set_tcp,
            movel,
            movej,
            movejx,
            task_compliance_ctrl,
            set_desired_force,
            get_current_posx,
            release_force,
            release_compliance_ctrl,
            DR_BASE,
            DR_MV_MOD_REL,
            DR_FC_MOD_REL,
            wait,
        )
        from DR_common2 import posx, posj

        dsr_functions = {
            "set_digital_output": set_digital_output,
            "set_tool": set_tool,
            "set_tcp": set_tcp,
            "movel": movel,
            "movej": movej,
            "movejx": movejx,
            "task_compliance_ctrl": task_compliance_ctrl,
            "set_desired_force": set_desired_force,
            "get_current_posx": get_current_posx,
            "release_force": release_force,
            "release_compliance_ctrl": release_compliance_ctrl,
            "wait" : wait,
        }

        dsr_constants = {
            "DR_BASE": DR_BASE,
            "DR_MV_MOD_REL": DR_MV_MOD_REL,
            "DR_FC_MOD_REL": DR_FC_MOD_REL,
        }

        robot = PourPills(
            node=node,
            dsr_functions=dsr_functions,
            dsr_constants=dsr_constants,
            posx=posx,
            posj=posj,
        )

        robot.run()

    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().info("Keyboard Interrupt")

    except Exception as error:
        if node is not None:
            node.get_logger().error(
                f"약 붓기 및 약통 버리기 테스트 실패: {error}"
            )
        else:
            print(f"약 붓기 및 약통 버리기 테스트 실패: {error}")

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
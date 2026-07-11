import json
import math
from time import sleep

import rclpy
import DR_init
import requests
from std_msgs.msg import String


ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

TCP_NAME = "Tool_v1"
TOOL_NAME = "Tool Weight_1"

VELOCITY = 50
ACC = 50

ON = 1
OFF = 0

MEDICINE_TOPIC = "/dsr01/pharmacy/medicine"
DONE_URL = "http://172.23.0.129:8000/api/tasks/refill/"

# --------------------------------------------------
# 약 붓기용 가상 TCP 설정
# --------------------------------------------------
# 현재 활성 TCP 원점에서 약병 끝점까지의 로컬 좌표 [mm]
# TCP +Y 방향 22.5 mm, TCP +Z 방향 25.0 mm
# 거치대 끝 [0.0, 29, 10]
# 테스트용 실제에서는 db에서 받아올 거임
# BOTTLE_TIP_OFFSET_TCP = [0.0, 25, -42]

# 현재 TCP의 로컬 X축을 중심으로 회전

POUR_TOTAL_ANGLE_DEG = +60.0
POUR_STEP_DEG = 10.0
POUR_VEL =15
POUR_ACC = 15
POUR_HOLD_SEC = 1.0


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
        posj
    ):
        # main()에서 생성한 ROS2 노드
        self.node = node

        # --------------------------------------------------
        # DSR 함수
        # --------------------------------------------------
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

        # --------------------------------------------------
        # DSR 상수
        # --------------------------------------------------
        self.DR_BASE = dsr_constants["DR_BASE"]
        self.DR_MV_MOD_REL = dsr_constants["DR_MV_MOD_REL"]

        self.posx = posx
        self.posj = posj

        self.vel = VELOCITY
        self.acc = ACC

        # --------------------------------------------------
        # 현재 작업 정보
        # --------------------------------------------------
        self.current_task = None
        self.task_queue = []

        self.medicine_id = None
        self.medicine_name = None
        self.refill_amount = None
        self.lid_type = None

        self.storage_stock = None
        self.dispensing_stock = None

        # --------------------------------------------------
        # 조제기 위치
        # --------------------------------------------------
        self.dispensing_x = None
        self.dispensing_y = None
        self.dispensing_z = None
        self.dispensing_rx = None
        self.dispensing_ry = None
        self.dispensing_rz = None

        # --------------------------------------------------
        # 약통 tcp (Tool_v1 기준)
        # --------------------------------------------------
        self.tcp_x = None
        self.tcp_y = None
        self.tcp_z = None

        # --------------------------------------------------
        # 서랍 위치
        # --------------------------------------------------
        self.drawer_x = None
        self.drawer_y = None
        self.drawer_z = None
        self.drawer_rx = None
        self.drawer_ry = None
        self.drawer_rz = None

        # --------------------------------------------------
        # 로봇 위치
        # --------------------------------------------------
        self.X_DISPENSER_POS = None
        self.X_DRAWER = None
        self.X_LOCK_RETURN = None

        self.X_TRASH_DROP = None
        self.X_TWEEZER_HOME = None
        self.X_TWEEZER_NEAR = None

        # 실제 의미는 서랍을 연 후의 로봇 위치
        self.X_DRAWER_CLOSED = None

        self.define_positions()
        self.create_subscriptions()
        self.init_robot()

    # --------------------------------------------------
    # Logger
    # --------------------------------------------------
    def get_logger(self):
        return self.node.get_logger()

    # --------------------------------------------------
    # 0. 고정 위치 정의
    # --------------------------------------------------
    def define_positions(self):
        self.X_TRASH_DROP = self.posx(
            -423.12,
            -96.72,
            89.41,
            8.51,
            -179.18,
            101.63,
        )

    # --------------------------------------------------
    # 1. 약품 데이터 구독
    # --------------------------------------------------
    def create_subscriptions(self):
        self.sub_medicine = self.node.create_subscription(
            String,
            MEDICINE_TOPIC,
            self.dispenser_pose_callback,
            10,
        )

        self.get_logger().info(
            f"약품 작업 토픽 구독 시작: {MEDICINE_TOPIC}"
        )

    def dispenser_pose_callback(self, msg):
        try:
            data = json.loads(msg.data)

            if not isinstance(data, list):
                raise ValueError(
                    "수신 데이터는 list 형식이어야 함"
                )

            if len(data) == 0:
                raise ValueError(
                    "수신 데이터가 비어 있음"
                )

            self.task_queue.extend(data)

            self.get_logger().info(
                f"작업 {len(data)}개 추가 수신, "
                f"현재 대기 작업 {len(self.task_queue)}개"
            )

        except json.JSONDecodeError as e:
            self.get_logger().error(
                f"JSON 변환 실패: {e}"
            )

        except Exception as e:
            self.get_logger().error(
                f"작업 데이터 수신 실패: {e}"
            )

    # --------------------------------------------------
    # 2. 수신 데이터에서 현재 작업 설정
    # --------------------------------------------------
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

            "bottle_tip_offset_x",
            "bottle_tip_offset_y",
            "bottle_tip_offset_z",

            "drawer_x",
            "drawer_y",
            "drawer_z",
            "drawer_rx",
            "drawer_ry",
            "drawer_rz",

            "lid_type",
            "storage_stock",
            "dispensing_stock",
        ]

        for key in required_keys:
            if key not in task:
                raise KeyError(
                    f"필수 데이터가 없음: {key}"
                )

        self.current_task = task

        self.medicine_id = int(
            task["medicine_number"]
        )

        self.medicine_name = str(
            task["medicine_name"]
        )

        # 조제기 좌표
        self.dispensing_x = float(task["dispensing_x"])
        self.dispensing_y = float(task["dispensing_y"])
        self.dispensing_z = float(task["dispensing_z"])
        self.dispensing_rx = float(task["dispensing_rx"])
        self.dispensing_ry = float(task["dispensing_ry"])
        self.dispensing_rz = float(task["dispensing_rz"])

        # 서랍 좌표
        self.drawer_x = float(task["drawer_x"])
        self.drawer_y = float(task["drawer_y"])
        self.drawer_z = float(task["drawer_z"])
        self.drawer_rx = float(task["drawer_rx"])
        self.drawer_ry = float(task["drawer_ry"])
        self.drawer_rz = float(task["drawer_rz"])

        self.lid_type = str(task["lid_type"]).strip().lower()

        self.storage_stock = int(task["storage_stock"])

        self.dispensing_stock = int(task["dispensing_stock"])

        # 현재 코드는 저장 약품 전부를 리필량으로 사용
        self.refill_amount = self.storage_stock

        if self.refill_amount <= 0:
            raise ValueError(
                "storage_stock이 0 이하라 리필할 수 없음"
            )

        self.X_DISPENSER_POS = self.posj(
            self.dispensing_x,
            self.dispensing_y,
            self.dispensing_z,
            self.dispensing_rx,
            self.dispensing_ry,
            self.dispensing_rz,
        )

        self.X_DRAWER = self.posj(
            self.drawer_x,
            self.drawer_y,
            self.drawer_z,
            self.drawer_rx,
            self.drawer_ry,
            self.drawer_rz,
        )

        self.bottle_tip_offset_tcp = [
        float(task["bottle_tip_offset_x"]),
        float(task["bottle_tip_offset_y"]),
        float(task["bottle_tip_offset_z"]),
    ]

        self.get_logger().info(
            f"현재 작업 설정 완료: "
            f"medicine_number={self.medicine_id}, "
            f"medicine_name={self.medicine_name}, "
            f"lid_type={self.lid_type}"
        )

    def wait_for_task(self):
        self.get_logger().info(
            "작업 데이터 대기 중"
        )

        while rclpy.ok() and not self.task_queue:
            rclpy.spin_once(
                self.node,
                timeout_sec=0.1,
            )

        if not self.task_queue:
            raise RuntimeError(
                "작업 데이터 수신 실패"
            )

    # --------------------------------------------------
    # 3. 로봇 초기 세팅
    # --------------------------------------------------
    def init_robot(self):
        self.get_logger().info(
            "로봇 초기 세팅 시작"
        )

        # self.set_tool("Tool Weight_1")
        # self.set_tcp("Tool_v1")

        self.release()

        self.get_logger().info(
            "로봇 초기 세팅 완료"
        )

    # --------------------------------------------------
    # 4. 그리퍼
    # --------------------------------------------------
    def release(self):
        self.get_logger().info(
            "그리퍼 열기"
        )

        self.set_digital_output(1, OFF)
        self.set_digital_output(2, OFF)

        sleep(0.2)

        self.set_digital_output(1, ON)
        self.set_digital_output(2, OFF)

        sleep(1)

    def grip(self):
        self.get_logger().info(
            "그리퍼 닫기"
        )

        self.set_digital_output(1, OFF)
        self.set_digital_output(2, OFF)

        sleep(0.2)

        self.set_digital_output(1, OFF)
        self.set_digital_output(2, ON)

        sleep(1)

    def little(self):
        self.get_logger().info(
            "그리퍼 살짝 닫기"
        )

        self.set_digital_output(1, OFF)
        self.set_digital_output(2, OFF)

        sleep(0.2)

        self.set_digital_output(1, ON)
        self.set_digital_output(2, ON)

        sleep(1)

    # --------------------------------------------------
    # 5. 서랍 열기 유후
    # --------------------------------------------------
    def open_drawer(self):
        if self.X_DRAWER is None:
            raise RuntimeError(
                "서랍 열기 위치가 설정되지 않음"
            )

        self.get_logger().info(
            "서랍 열기 시작"
        )

        self.movej(
            self.X_DRAWER,
            vel=self.vel,
            acc=self.acc,
            ref=self.DR_BASE
        )

        self.grip()

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

        current_result = self.get_current_posx(
            ref=self.DR_BASE
        )

        if current_result is None:
            raise RuntimeError(
                "현재 위치 조회 결과가 None임"
            )

        current_pos = current_result[0]

        if current_pos is None or len(current_pos) < 6:
            raise RuntimeError(
                f"현재 위치 데이터가 올바르지 않음: "
                f"{current_result}"
            )

        x, y, z, rx, ry, rz = current_pos[:6]

        # 이름은 기존 코드와 동일하게 유지
        # 실제로는 서랍이 열린 상태의 로봇 위치
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

    # --------------------------------------------------
    # 6. 도구 위치로 이동
    # --------------------------------------------------
    def go_to_tool(self):
        # 나중에 코드 합치고 나서는 없어질 예정 단독 테스트 용으로 놔둠 
        self.X_LOCK_RETURN = self.posj(-0.14, 30.09, 77.46, -0.33, 73.14, -7.15)

        if self.X_LOCK_RETURN is None:
            raise RuntimeError(
                "X_LOCK_RETURN 좌표가 설정되지 않음"
            )

        self.movej(self.X_LOCK_RETURN, vel=self.vel, acc=self.acc)
        # # 나중에 코드 합치고 나서는 movejx 쓸거임 
        # self.movejx(self.X_LOCK_RETURN, vel=self.vel, acc=self.acc, sol=2)

        self.movej(
            self.posj(-0.30, 29.88, 77.79, -0.05, 72.95, 13.42),
            vel=self.vel,
            acc=self.acc
        )
        # 혜승님 코드 베껴서 몇도돌릴지 보고 movel로 바꾸기 

        sleep(0.5)

        start_pose = self.get_current_pos_base()
        start_z = start_pose[2]

        max_lift_distance = 22.0
        
        try:
            self.task_compliance_ctrl([10000, 10000, 500, 300, 300, 300])
            self.set_desired_force(fd=[0,0,30,0,0,0], dir=[0,0,1,0,0,0])
            
            while True:
                current_pose = self.get_current_pos_base()
                current_z = current_pose[2]
                lifted_distance = current_z - start_z
                if lifted_distance >= max_lift_distance:
                    self.get_logger().info("다 올라옴 그만")
                    break
        
        finally:
            self.release_force()
            self.release_compliance_ctrl()

    # --------------------------------------------------
    # 7. 조제기로 이동
    # --------------------------------------------------
    def move_pour(self):
        if self.X_DISPENSER_POS is None:
            raise RuntimeError(
                "조제기 위치가 설정되지 않음"
            )

        self.get_logger().info(
            "조제기 위치로 이동 시작"
        )

        self.movej(
            self.X_DISPENSER_POS,
            vel=self.vel,
            acc=self.acc,
            ref=self.DR_BASE
        )

        self.pour_tweezer()

        self.get_logger().info(
            "조제기 위치 작업 완료"
        )

    # --------------------------------------------------
    # 9. 약 붓기: 현재 TCP에서 떨어진 약병 끝점을 가상 회전축으로 사용
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
        """두산 기본 자세각 Z-Y'-Z''를 회전행렬로 변환."""
        rz_a = cls._rotation_z(math.radians(a_deg))
        ry_b = cls._rotation_y(math.radians(b_deg))
        rz_c = cls._rotation_z(math.radians(c_deg))
        return cls._matmul_3x3(cls._matmul_3x3(rz_a, ry_b), rz_c)

    @staticmethod
    def _angle_near_reference(angle_deg, reference_deg):
        """동일 각도 표현 중 reference_deg와 가장 가까운 값 선택."""
        return angle_deg + 360.0 * round((reference_deg - angle_deg) / 360.0)

    @classmethod
    def _rotation_matrix_to_zyz(cls, rotation, reference_abc):
        """
        회전행렬을 ZYZ 각도로 변환.

        ZYZ는 같은 자세를 여러 각도 조합으로 표현할 수 있으므로,
        직전 자세(reference_abc)와 가장 가까운 표현을 선택한다.
        """
        r22 = max(-1.0, min(1.0, rotation[2][2]))
        b = math.acos(r22)
        sin_b = math.sin(b)
        epsilon = 1e-9

        if abs(sin_b) > epsilon:
            a = math.atan2(rotation[1][2], rotation[0][2])
            c = math.atan2(rotation[2][1], -rotation[2][0])
        elif r22 > 0.0:
            # B ~= 0: A+C만 결정되므로 C=0인 표현 사용
            b = 0.0
            a = math.atan2(rotation[1][0], rotation[0][0])
            c = 0.0
        else:
            # B ~= pi: 특이점에서 가능한 표현 하나를 사용
            b = math.pi
            a = math.atan2(-rotation[1][0], -rotation[0][0])
            c = 0.0

        candidate_1 = [
            math.degrees(a),
            math.degrees(b),
            math.degrees(c),
        ]

        # Rz(A) Ry(B) Rz(C)
        # == Rz(A+180) Ry(-B) Rz(C+180)
        candidate_2 = [
            candidate_1[0] + 180.0,
            -candidate_1[1],
            candidate_1[2] + 180.0,
        ]

        candidates = []
        for candidate in (candidate_1, candidate_2):
            adjusted = [
                cls._angle_near_reference(candidate[i], reference_abc[i])
                for i in range(3)
            ]
            score = sum(
                (adjusted[i] - reference_abc[i]) ** 2
                for i in range(3)
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

        result = []
        angle = signed_step

        while abs(angle) < abs(total_angle_deg):
            result.append(angle)
            angle += signed_step

        result.append(total_angle_deg)
        return result

    def _get_current_pose_zyz(self):
        current_result = self.get_current_posx(
            ref=self.DR_BASE,
        )

        if current_result is None:
            raise RuntimeError("현재 위치 조회 실패")

        # 일반적인 반환은 (posx, solution_space)지만,
        # 라이브러리 버전에 따라 posx 자체가 반환되는 경우도 처리한다.
        if (
            hasattr(current_result, "__len__")
            and len(current_result) >= 6
            and isinstance(current_result[0], (int, float))
        ):
            current_pos = current_result
        elif hasattr(current_result, "__len__") and len(current_result) >= 1:
            current_pos = current_result[0]
        else:
            current_pos = None

        if current_pos is None or len(current_pos) < 6:
            raise RuntimeError(
                f"현재 위치 데이터가 올바르지 않음: {current_result}"
            )

        return [float(value) for value in current_pos[:6]]

    def _calculate_virtual_tcp_pose(
        self,
        start_rotation,
        tip_position_base,
        angle_deg,
        reference_abc,
    ):
        """
        약병 끝점은 BASE 좌표에서 고정하고,
        현재 TCP의 로컬 X축으로 angle_deg만큼 회전한 절대 posx를 계산한다.
        """
        local_x_rotation = self._rotation_x(math.radians(angle_deg))
        target_rotation = self._matmul_3x3(
            start_rotation,
            local_x_rotation,
        )

        rotated_tip_offset_base = self._matvec_3x3(
            target_rotation,
            BOTTLE_TIP_OFFSET_TCP,
        )

        target_position = [
            tip_position_base[i] - rotated_tip_offset_base[i]
            for i in range(3)
        ]

        target_abc = self._rotation_matrix_to_zyz(
            target_rotation,
            reference_abc,
        )

        target_values = target_position + target_abc
        target_pose = self.posx(
            *target_values
        )

        return target_pose, target_values, target_abc

    def pour_tweezer(self):
        """
        활성 TCP를 변경하지 않고 약병 끝점 중심으로 붓는다.

        전제:
        - 활성 TCP에서 약병 끝점까지 오프셋이
          BOTTLE_TIP_OFFSET_TCP와 동일해야 한다.
        - 회전축은 활성 TCP와 평행한 가상 TCP의 로컬 X축이다.
        """
        start_pose = self._get_current_pose_zyz()
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
                    start_pose=start_pose,
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

        # 계산했던 절대 자세를 역순으로 따라가며 복귀한다.
        for target_pose in reversed(calculated_poses[:-1]):
            self.movel(
                target_pose,
                vel=POUR_VEL,
                acc=POUR_ACC,
                ref=self.DR_BASE,
            )

        # 수치 오차 없이 최초 측정 자세로 정확히 복귀
        self.movel(
            self.posx(
                *start_pose,
            ),
            vel=POUR_VEL,
            acc=POUR_ACC,
            ref=self.DR_BASE,
        )

        self.get_logger().info("약 붓기 완료")
        self.notify_done()

    # --------------------------------------------------
    # 10. 약통 버리기
    # --------------------------------------------------
    def move_trash(self):
        if self.X_TRASH_DROP is None:
            raise RuntimeError(
                "쓰레기통 위치가 설정되지 않음"
            )

        self.get_logger().info(
            "약통 버리기 시작"
        )

        self.movejx(
            self.X_TRASH_DROP,
            vel=self.vel,
            acc=self.acc,
            ref=self.DR_BASE,
            sol=2,
        )

        self.release()

        self.get_logger().info(
            "약통 버리기 완료"
        )


    # --------------------------------------------------
    # 12. 서랍 닫기
    # --------------------------------------------------
    def close_drawer(self):
        if self.X_DRAWER_CLOSED is None:
            raise RuntimeError(
                "서랍을 연 후 위치가 저장되지 않음"
            )

        if self.X_DRAWER is None:
            raise RuntimeError(
                "서랍 위치가 설정되지 않음"
            )

        self.get_logger().info(
            "서랍 닫기 시작"
        )

        self.movejx(
            self.X_DRAWER_CLOSED,
            vel=self.vel,
            acc=self.acc,
            ref=self.DR_BASE,
            sol=2,
        )

        self.movel(
            self.posx(
                125,
                0,
                0,
                0,
                0,
                0,
            ),
            vel=10,
            acc=10,
            ref=self.DR_BASE,
            mod=self.DR_MV_MOD_REL,
        )

        self.get_logger().info(
            "서랍 닫기 완료"
        )

    # --------------------------------------------------
    # 13. DB 리필 완료 알림
    # --------------------------------------------------
    def notify_done(self):
        if self.medicine_name is None:
            raise RuntimeError(
                "medicine_name이 설정되지 않음"
            )

        if self.refill_amount is None:
            raise RuntimeError(
                "refill_amount가 설정되지 않음"
            )

        payload = {
            "medicine_name": self.medicine_name,
            "amount": self.refill_amount,
        }

        self.get_logger().info(
            f"리필 완료 POST 전송: {payload}"
        )

        try:
            response = requests.post(
                DONE_URL,
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
    # 14. Spin 타입 작업
    # --------------------------------------------------
    def run_gripper_lid_task(self):
        self.get_logger().info(
            "spin 방식 작업 시작"
        )

        self.open_drawer()
        self.go_to_tool()
        self.move_pour()
        self.move_trash()
        self.close_drawer()
        self.get_logger().info(
            "spin 방식 작업 완료"
        )

    # --------------------------------------------------
    # 15. Pull 타입 작업
    # --------------------------------------------------
    def run_hole_lid_task(self):
        self.get_logger().info(
            "pull 방식 작업 시작"
        )

        self.open_drawer()
        self.go_to_tool()
        self.move_pour()
        self.move_trash()
        self.close_drawer()
        self.get_logger().info(
            "pull 방식 작업 완료"
        )

    # --------------------------------------------------
    # 16. 전체 실행
    # --------------------------------------------------
    def run(self):
        self.get_logger().info(
            "전체 작업 시작"
        )

        self.wait_for_task()

        while rclpy.ok():
            while self.task_queue:
                task = self.task_queue.pop(0)

                try:
                    self.set_task_from_data(task)

                    self.get_logger().info(
                        f"작업 시작: "
                        f"{self.medicine_name}, "
                        f"lid_type={self.lid_type}, "
                        f"refill_amount={self.refill_amount}"
                    )

                    if self.lid_type == "pull":
                        self.run_hole_lid_task()

                    elif self.lid_type == "spin":
                        self.run_gripper_lid_task()

                    else:
                        raise ValueError(
                            f"지원하지 않는 lid_type: "
                            f"{self.lid_type}"
                        )

                    self.get_logger().info(
                        f"작업 완료: "
                        f"{self.medicine_name}"
                    )

                except Exception as e:
                    self.get_logger().error(
                        f"현재 작업 실행 실패: {e}"
                    )

                # 작업 중 들어온 ROS 메시지 처리
                rclpy.spin_once(
                    self.node,
                    timeout_sec=0.1,
                )

            self.get_logger().info(
                "대기 작업 없음. 새 작업 대기 중"
            )

            rclpy.spin_once(
                self.node,
                timeout_sec=0.5,
            )


def main(args=None):
    rclpy.init(args=args)

    node = None
    robot = None

    try:
        # --------------------------------------------------
        # 1. ROS2 노드 생성
        # --------------------------------------------------
        node = rclpy.create_node(
            "pour_pills",
            namespace=ROBOT_ID,
        )

        # --------------------------------------------------
        # 2. DSR_ROBOT2 import 전에 node 설정
        # --------------------------------------------------
        DR_init.__dsr__node = node

        node.get_logger().info(
            "DR_init 설정 확인: "
            f"id={DR_init.__dsr__id}, "
            f"model={DR_init.__dsr__model}, "
            f"node={DR_init.__dsr__node}"
        )

        # --------------------------------------------------
        # 3. DR_init 설정 완료 후 DSR_ROBOT2 import
        # --------------------------------------------------
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
            DR_BASE,
            DR_MV_MOD_REL,
        )

        from DR_common2 import posx, posj

        node.get_logger().info(
            "DSR_ROBOT2 import 완료"
        )

        # --------------------------------------------------
        # 4. 클래스에 함수와 상수 전달
        # --------------------------------------------------
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
        }

        dsr_constants = {
            "DR_BASE": DR_BASE,
            "DR_MV_MOD_REL": DR_MV_MOD_REL,
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
            node.get_logger().info(
                "Keyboard Interrupt"
            )

    except ImportError as e:
        if node is not None:
            node.get_logger().error(
                f"DSR_ROBOT2 import 실패: {e}"
            )
        else:
            print(
                f"DSR_ROBOT2 import 실패: {e}"
            )

    except Exception as e:
        if node is not None:
            node.get_logger().error(
                f"Robot error: {e}"
            )
        else:
            print(
                f"Robot initialization error: {e}"
            )

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
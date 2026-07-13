import json
import math
import os
import time
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

MEDICINE_TOPIC = "/dsr01/pharmacy/refill_required_medicine"
DONE_URL = os.getenv(
    "PHARMACY_DONE_URL",
    "http://127.0.0.1:8000/api/tasks/refill/",
)

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

# --------------------------------------------------
# 뚜껑 병따개 걸기용 가상 TCP 설정
# --------------------------------------------------
OPENER_HOOK_TOTAL_ANGLE_DEG = -30.0
OPENER_HOOK_STEP_DEG = 6.0
OPENER_HOOK_VEL = 5
OPENER_HOOK_ACC = 5
OPENER_HOOK_HOLD_SEC = 1.0
OPENER_TIP_OFFSET_TCP = [0.0, 0.0, 75.0]

# 약통을 보관 위치에서 빼낼 때 사용하는 순응제어/힘제어 설정
MEDICINE_LIFT_DISTANCE = 40.0
COMPLIANCE_STX = [10000, 10000, 700, 300, 300, 300]
DESIRED_FORCE = [0, 0, 50, 0, 0, 0]
FORCE_DIRECTION = [0, 0, 1, 0, 0, 0]
FORCE_CHECK_INTERVAL_SEC = 0.05

# 서랍 작업 전후와 약통 상승 후 사용하는 충돌 회피용 Joint 위치
DRAWER_SAFE_JOINT = [-42.63, -0.65, 99.42, -1.37, 81.22, 7.57]

# --------------------------------------------------
# 약통 파지 확인 설정
# --------------------------------------------------

# 실제 그리퍼 파지 완료 신호가 연결된 DI 번호로 변경
BOTTLE_GRIP_INPUT = 1

# 파지 성공일 때 입력값
BOTTLE_GRIP_OK_VALUE = ON

# 파지 신호를 기다릴 최대 시간
BOTTLE_GRIP_TIMEOUT_SEC = 2.0

# 입력 확인 주기
BOTTLE_GRIP_CHECK_INTERVAL_SEC = 0.05

# task manager가 같은 작업을 반복 publish할 때 재등록을 막는 시간
RECENT_TASK_IGNORE_SEC = 30.0


# 반드시 DSR_ROBOT2 import 전에 설정
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

class GraspError(RuntimeError):
    """약통 파지 관련 기본 예외."""
    pass


class GraspTimeoutError(GraspError):
    """약통을 잡지 못한 경우. 재시도 가능."""
    pass


class GraspSensorError(GraspError):
    """파지 센서 읽기 오류. 재시도 불가능."""
    pass

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
        self.wait = dsr_functions["wait"]

        # --------------------------------------------------
        # DSR 함수
        # --------------------------------------------------
        self.set_digital_output = dsr_functions["set_digital_output"]
        self.get_digital_input = dsr_functions["get_digital_input"]
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
        self.get_tool_force = dsr_functions["get_tool_force"]

        # --------------------------------------------------
        # DSR 상수
        # --------------------------------------------------
        self.DR_BASE = dsr_constants["DR_BASE"]
        self.DR_MV_MOD_REL = dsr_constants["DR_MV_MOD_REL"]
        self.DR_FC_MOD_REL = dsr_constants["DR_FC_MOD_REL"]
        self.DR_TOOL = dsr_constants["DR_TOOL"]

        self.posx = posx
        self.posj = posj

        self.vel = VELOCITY
        self.acc = ACC

        # --------------------------------------------------
        # 현재 작업 정보
        # --------------------------------------------------
        self.current_task = None
        self.task_queue = []

        # task manager가 1초마다 같은 배열을 publish하므로 중복 작업을 막는다.
        self.queued_task_keys = set()
        self.active_task_key = None
        self.recently_completed_task_times = {}

        self.medicine_id = None
        self.medicine_name = None
        self.refill_amount = None
        self.lid_type = None

        self.storage_stock = None
        self.dispensing_stock = None

        # --------------------------------------------------
        # 적재소 약통 위치
        # --------------------------------------------------
        self.storage_x = None
        self.storage_y = None
        self.storage_z = None
        self.storage_rx = None
        self.storage_ry = None
        self.storage_rz = None

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
        self.bottle_tip_offset_tcp = None

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
        self.X_STORAGE_LOC = None
        self.X_DISPENSER_POS = None
        self.X_DRAWER = None
        self.X_LOCK_RETURN = None
        self.X_OPENER_RETURN = None

        # 병따개 끝점 오프셋은 약 붓기용 DB 오프셋과 별도로 사용한다.
        self.opener_tip_offset_tcp = list(OPENER_TIP_OFFSET_TCP)

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
        # 적재소 근처 접근 위치
        self.X_STORAGE_APPROACH = self.posx(
            357.12, 219.79, 200.52,
            96.00, 176.96, 105.65,
        )

        # 약통 고정 거치대
        self.X_FIX_ABOVE = self.posx(
            550.90, 1.02, 200.52,
            8.33, -179.62, 18.10,
        )
        self.X_FIX = self.posx(
            551.90, 2.20, 50.91,
            8.33, -179.62, 18.10,
        )
        self.X_SPIN_LID_ABOVE = self.posx(
            551.90, 2.20, 100.91,
            8.33, -179.62, 18.10,
        )

        # 당겨서 여는 뚜껑용 병따개 위치
        self.X_OPENER_TOOL_ABOVE = self.posx(
            582.33, 244.77, 129.81,
            39.98, 180.00, 132.74,
        )
        self.X_OPENER_TOOL = self.posx(
            582.33, 244.77, 99.81,
            39.98, 180.00, 132.74,
        )
        self.X_OPEN_READY_ABOVE = self.posx(
            602.07, 6.05, 182.47,
            161.74, -179.88, 157.58,
        )
        self.X_OPEN_READY = self.posx(
            602.07, 6.05, 148.47,
            161.74, -179.88, 157.58,
        )

        # 뚜껑과 빈 약통이 사용하는 쓰레기통 위치
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
            1,
        )

        self.get_logger().info(
            f"약품 작업 토픽 구독 시작: {MEDICINE_TOPIC}"
        )

    @staticmethod
    def _make_task_key(task):
        """Task Manager와 동일하게 약 이름을 기준으로 중복 작업을 구분한다."""
        if not isinstance(task, dict):
            return None

        medicine_name = task.get("medicine_name")
        if isinstance(medicine_name, str):
            medicine_name = medicine_name.strip()
            if medicine_name:
                return f"medicine_name:{medicine_name}"

        return None

    def _purge_old_completed_task_keys(self):
        now = time.monotonic()
        expired = [
            key
            for key, completed_at in self.recently_completed_task_times.items()
            if now - completed_at >= RECENT_TASK_IGNORE_SEC
        ]
        for key in expired:
            del self.recently_completed_task_times[key]

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

            self._purge_old_completed_task_keys()

            added_count = 0
            skipped_count = 0

            for task in data:
                task_key = self._make_task_key(task)
                if task_key is None:
                    self.get_logger().warning(
                        f"작업 식별 키가 없어 건너뜀: {task}"
                    )
                    skipped_count += 1
                    continue

                if (
                    task_key in self.queued_task_keys
                    or task_key == self.active_task_key
                    or task_key in self.recently_completed_task_times
                ):
                    skipped_count += 1
                    continue

                self.task_queue.append(task)
                self.queued_task_keys.add(task_key)
                added_count += 1

            self.get_logger().info(
                f"작업 수신: 추가={added_count}, 중복 생략={skipped_count}, "
                f"현재 대기={len(self.task_queue)}"
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

            "lid_type",
            "storage_stock",
            "dispensing_stock",
        ]

        for key in required_keys:
            if key not in task:
                raise KeyError(f"필수 데이터가 없음: {key}")

        self.current_task = task

        self.medicine_id = int(task["medicine_number"])

        self.medicine_name = str(task["medicine_name"])

        # 적재소 약통 좌표는 직교 좌표(posx)로 사용한다.
        self.storage_x = float(task["storage_x"])
        self.storage_y = float(task["storage_y"])
        self.storage_z = float(task["storage_z"])
        self.storage_rx = float(task["storage_rx"])
        self.storage_ry = float(task["storage_ry"])
        self.storage_rz = float(task["storage_rz"])
        self.X_STORAGE_LOC = self.posx(
            self.storage_x,
            self.storage_y,
            self.storage_z,
            self.storage_rx,
            self.storage_ry,
            self.storage_rz,
        )

        # 조제기 좌표
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

        # 서랍 좌표
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

        self.lid_type = str(task["lid_type"]).strip().lower()
        self.storage_stock = int(task["storage_stock"])
        self.dispensing_stock = int(task["dispensing_stock"])

        # 현재 코드는 저장 약품 전부를 리필량으로 사용
        self.refill_amount = self.storage_stock
        if self.refill_amount <= 0:
            raise ValueError("storage_stock이 0 이하라 리필할 수 없음")

        self.bottle_tip_offset_tcp = [
            float(task["bottle_tip_offset_x"]),
            float(task["bottle_tip_offset_y"]),
            float(task["bottle_tip_offset_z"]),
        ]

        self.get_logger().info(
            f"현재 작업 설정 완료: "
            f"medicine_number={self.medicine_id}, "
            f"medicine_name={self.medicine_name}, "
            f"lid_type={self.lid_type}, "
            f"storage_posx={list(self.X_STORAGE_LOC)}, "
            f"dispensing_posj={list(self.X_DISPENSER_POS)}, "
            f"drawer_posj={list(self.X_DRAWER)}, "
            f"bottle_tip_offset_tcp={self.bottle_tip_offset_tcp}"
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
        self.get_logger().info("로봇 초기 세팅 시작")

        self.set_tool("Tool Weight_1")
        self.set_tcp("Tool_v1")

        self.release()

        self.get_logger().info("로봇 초기 세팅 완료")

    # --------------------------------------------------
    # 4. 그리퍼
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

    def little_grip(self):
        self.get_logger().info("그리퍼 살짝 닫기")

        self.set_digital_output(1, OFF)
        self.set_digital_output(2, OFF)
        sleep(0.2)

        self.set_digital_output(1, ON)
        self.set_digital_output(2, ON)
        sleep(1)
    
    def wait_for_bottle_grip(
        self,
        timeout_sec=BOTTLE_GRIP_TIMEOUT_SEC,
    ):
        """
        약통 파지 완료 디지털 입력을 timeout 동안 확인한다.

        성공:
            True 반환

        실패:
            GraspError 발생
        """
        start_time = time.monotonic()

        while rclpy.ok():
            input_value = self.get_digital_input(
                BOTTLE_GRIP_INPUT
            )

            if input_value is None:
                raise GraspSensorError(
                    "약통 파지 입력이 None으로 반환됨: "
                    f"DI={BOTTLE_GRIP_INPUT}"
                )

            if input_value not in (OFF, ON):
                raise GraspSensorError(
                    "약통 파지 입력값이 올바르지 않음: "
                    f"DI={BOTTLE_GRIP_INPUT}, "
                    f"value={input_value}"
                )

            if input_value == BOTTLE_GRIP_OK_VALUE:
                elapsed = time.monotonic() - start_time

                self.get_logger().info(
                    "약통 파지 신호 확인 완료: "
                    f"DI={BOTTLE_GRIP_INPUT}, "
                    f"value={input_value}, "
                    f"elapsed={elapsed:.2f} sec"
                )
                return

            elapsed = time.monotonic() - start_time

            if elapsed >= timeout_sec:
                raise GraspTimeoutError(
                    "약통 파지 신호 시간 초과: "
                    f"medicine={self.medicine_name}, "
                    f"DI={BOTTLE_GRIP_INPUT}, "
                    f"last_value={input_value}, "
                    f"timeout={timeout_sec:.1f} sec"
                )

            sleep(BOTTLE_GRIP_CHECK_INTERVAL_SEC)

        raise GraspError(
            "ROS2 종료로 약통 파지 확인이 중단됨"
        )
        
    # --------------------------------------------------
    # 5. get_current_posx() 반환 형식 처리
    # --------------------------------------------------
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
        
    # --------------------------------------------------
    # 5. 서랍 열기 유후
    # --------------------------------------------------
    def open_drawer(self):
        if self.X_DRAWER is None:
            raise RuntimeError("서랍 열기 위치가 설정되지 않음")

        self.get_logger().info("서랍 열기 시작")

        self.movej(
            self.posj(*DRAWER_SAFE_JOINT),
            vel=20,
            acc=20,
        )

        self.movej(
            self.X_DRAWER,
            vel=self.vel,
            acc=self.acc,
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
            self.posj(*DRAWER_SAFE_JOINT),
            vel=20,
            acc=20,
        ) 
    
    # --------------------------------------------------
    # 6. opener 시퀀스: 적재소 약통 파지
    # --------------------------------------------------
    def storage_grasp(self):
        if self.X_STORAGE_LOC is None:
            raise RuntimeError("적재소 약통 위치가 설정되지 않음")

        self.get_logger().info("=== 적재소 약통 파지 시퀀스 ===")

        self.movejx(
            self.X_STORAGE_APPROACH,
            vel=self.vel,
            acc=self.acc,
            ref=self.DR_BASE,
            sol=2,
        )
        self.movejx(
            self.X_STORAGE_LOC,
            vel=self.vel,
            acc=self.acc,
            ref=self.DR_BASE,
            sol=2,
        )

        self.movel(
            self.posx(0, 0, -27, 0, 0, 0),
            vel=20,
            acc=20,
            ref=self.DR_BASE,
            mod=self.DR_MV_MOD_REL,
        )
        self.grip()

        self.movel(
            self.posx(0, 0, 40, 0, 0, 0),
            vel=20,
            acc=20,
            ref=self.DR_BASE,
            mod=self.DR_MV_MOD_REL,
        )
        self.wait(0.5)
        self.movejx(
            self.X_STORAGE_APPROACH,
            vel=self.vel,
            acc=self.acc,
            ref=self.DR_BASE,
            sol=2,
        )

    # --------------------------------------------------
    # opener 시퀀스: 약통을 거치대에 가압 삽입
    # --------------------------------------------------
    def pull_down(self):
        self.get_logger().info("거치대에 약통 가압 삽입 시작")

        compliance_started = False
        force_started = False
        target_z = 29.93
        z_tolerance = 1.0
        start_time = time.monotonic()

        try:
            self.task_compliance_ctrl(
                stx=[500, 500, 100, 5000, 5000, 5000]
            )
            compliance_started = True
            self.wait(0.5)

            self.set_desired_force(
                fd=[0, 0, -35, 0, 0, 0],
                dir=[0, 0, 1, 0, 0, 0],
                mod=self.DR_FC_MOD_REL,
            )
            force_started = True
            self.wait(0.5)

            while rclpy.ok():
                current_z = self.get_current_pos_base()[2]

                if abs(current_z - target_z) <= z_tolerance:
                    self.get_logger().info(
                        f"거치대 안착 완료: current_z={current_z:.2f}"
                    )
                    return True

                if time.monotonic() - start_time > 5.0:
                    self.get_logger().error(
                        "거치대 안착 시간 초과. 거치대 상태를 확인해야 함"
                    )
                    return False

                sleep(0.02)

            return False

        finally:
            if force_started:
                self.release_force()
            if compliance_started:
                self.release_compliance_ctrl()

    # --------------------------------------------------
    # opener 시퀀스: 약통 거치대 락킹
    # --------------------------------------------------
    def lock_bottle_in_fixture(self):
        self.get_logger().info("반시계 방향 회전으로 약통 거치대 락킹")

        # 원본 opener의 movej는 동기식이므로 회전 완료 후 반환된다.
        self.movej(
            self.posj(0, 0, 0, 0, 0, -17),
            vel=3,
            acc=1,
            mod=self.DR_MV_MOD_REL,
        )
        self.wait(0.5)
        return True

    def fix_lid(self):
        self.get_logger().info("=== 약통 거치대 고정 시퀀스 ===")

        self.movejx(
            self.X_FIX_ABOVE,
            vel=30,
            acc=30,
            ref=self.DR_BASE,
            sol=2,
        )
        self.wait(1.0)

        self.movejx(
            self.X_FIX,
            vel=5,
            acc=5,
            ref=self.DR_BASE,
            sol=2,
        )
        self.wait(0.5)

        if not self.pull_down():
            self.movel(
                self.posx(0, 0, 50, 0, 0, 0),
                vel=20,
                acc=20,
                ref=self.DR_BASE,
                mod=self.DR_MV_MOD_REL,
            )
            raise RuntimeError("약통 거치대 안착 실패")

        if not self.lock_bottle_in_fixture():
            raise RuntimeError("약통 거치대 락킹 실패")

        current = self.get_current_pos_base()
        self.X_LOCK_RETURN = self.posx(*current)
        self.get_logger().info(
            "약통 재파지 좌표 저장 완료: "
            f"{[round(value, 2) for value in current]}"
        )

        self.release()
        self.movel(
            self.posx(0, 0, 100, 0, 0, 0),
            vel=20,
            acc=20,
            ref=self.DR_BASE,
            mod=self.DR_MV_MOD_REL,
        )

    # --------------------------------------------------
    # pull 뚜껑: 접촉 위치 탐색
    # --------------------------------------------------
    def pull_lid_open(self):
        self.movejx(
            self.X_OPEN_READY_ABOVE,
            vel=10,
            acc=10,
            ref=self.DR_BASE,
            sol=2,
        )
        self.wait(0.5)

        self.movel(
            self.X_OPEN_READY,
            vel=5,
            acc=5,
            ref=self.DR_BASE,
        )
        self.wait(0.5)

        self.get_logger().info("병따개 X축 접촉 탐색 시작")
        last_force_x = 0.0

        for _ in range(100):
            current_force = self.get_tool_force(ref=self.DR_BASE)
            last_force_x = float(current_force[0])

            if last_force_x > 0.0:
                self.get_logger().info(
                    f"뚜껑 접촉 감지: Fx={last_force_x:.2f} N"
                )
                return True

            self.movel(
                self.posx(-2.0, 0, 0, 0, 0, 0),
                vel=5,
                acc=5,
                ref=self.DR_BASE,
                mod=self.DR_MV_MOD_REL,
            )
            sleep(0.05)

        self.get_logger().error(
            "뚜껑 접촉 탐색 실패: "
            f"last_Fx={last_force_x:.2f} N"
        )
        return False

    def _calculate_virtual_tcp_pose_about_y(
        self,
        start_rotation,
        tip_position_base,
        angle_deg,
        reference_abc,
    ):
        local_y_rotation = self._rotation_y(math.radians(angle_deg))
        target_rotation = self._matmul_3x3(
            start_rotation,
            local_y_rotation,
        )
        rotated_tip_offset_base = self._matvec_3x3(
            target_rotation,
            self.opener_tip_offset_tcp,
        )
        target_position = [
            tip_position_base[index] - rotated_tip_offset_base[index]
            for index in range(3)
        ]
        target_abc = self._rotation_matrix_to_zyz(
            target_rotation,
            reference_abc,
        )
        target_values = target_position + target_abc
        return self.posx(*target_values), target_values, target_abc

    # --------------------------------------------------
    # pull 뚜껑: 병따개 끝점을 중심으로 걸기
    # --------------------------------------------------
    def opener_tweezer(self):
        start_pose = self.get_current_pos_base()
        start_position = start_pose[:3]
        start_abc = start_pose[3:6]
        start_rotation = self._zyz_to_rotation_matrix(*start_abc)

        tip_offset_base = self._matvec_3x3(
            start_rotation,
            self.opener_tip_offset_tcp,
        )
        tip_position_base = [
            start_position[index] + tip_offset_base[index]
            for index in range(3)
        ]

        reference_abc = start_abc
        angles = self._make_angle_sequence(
            OPENER_HOOK_TOTAL_ANGLE_DEG,
            OPENER_HOOK_STEP_DEG,
        )

        for angle_deg in angles:
            target_pose, target_values, reference_abc = (
                self._calculate_virtual_tcp_pose_about_y(
                    start_rotation=start_rotation,
                    tip_position_base=tip_position_base,
                    angle_deg=angle_deg,
                    reference_abc=reference_abc,
                )
            )
            self.get_logger().info(
                f"병따개 걸기 {angle_deg:.1f}도 목표: "
                f"{[round(value, 3) for value in target_values]}"
            )
            self.movel(
                target_pose,
                vel=OPENER_HOOK_VEL,
                acc=OPENER_HOOK_ACC,
                ref=self.DR_BASE,
            )

        sleep(OPENER_HOOK_HOLD_SEC)
        self.get_logger().info("병따개를 뚜껑에 거는 동작 완료")
        return True

    def _return_opener_safely(self):
        if self.X_OPENER_RETURN is None:
            raise RuntimeError("병따개 반환 위치가 저장되지 않음")

        self.movejx(
            self.X_OPENER_RETURN,
            vel=30,
            acc=30,
            ref=self.DR_BASE,
            sol=0,
        )
        self.movel(
            self.posx(0, 0, -130, 0, 0, 0),
            vel=15,
            acc=15,
            ref=self.DR_BASE,
            mod=self.DR_MV_MOD_REL,
        )
        self.release()
        self.movel(
            self.posx(0, 0, 130, 0, 0, 0),
            vel=30,
            acc=30,
            ref=self.DR_BASE,
            mod=self.DR_MV_MOD_REL,
        )

    def pull_lid(self):
        self.get_logger().info("=== 당겨서 여는 약통 뚜껑 열기 ===")

        self.movejx(
            self.X_OPENER_TOOL_ABOVE,
            vel=30,
            acc=30,
            ref=self.DR_BASE,
            sol=2,
        )
        self.movel(
            self.X_OPENER_TOOL,
            vel=15,
            acc=15,
            ref=self.DR_BASE,
        )
        self.grip()

        self.movel(
            self.posx(0, 0, 130, 0, 0, 0),
            vel=30,
            acc=30,
            ref=self.DR_BASE,
            mod=self.DR_MV_MOD_REL,
        )
        self.X_OPENER_RETURN = self.posx(*self.get_current_pos_base())

        if not self.pull_lid_open():
            self._return_opener_safely()
            return False

        # opener.py의 잘못된 pour_tweezer() 호출을 opener_tweezer()로 수정.
        if not self.opener_tweezer():
            self._return_opener_safely()
            return False

        self.movel(
            self.posx(0, 0, -150, 0, 0, 0),
            vel=60,
            acc=60,
            ref=self.DR_TOOL,
            mod=self.DR_MV_MOD_REL,
        )
        self._return_opener_safely()
        return True

    # --------------------------------------------------
    # spin 뚜껑 열기
    # --------------------------------------------------
    def spin_open(self):
        self.get_logger().info("라쳇 방식 spin 뚜껑 열기 시작")

        max_attempts = 5
        compliance_started = False
        force_started = False

        try:
            self.task_compliance_ctrl(
                stx=[10000, 10000, 300, 10000, 10000, 10000]
            )
            compliance_started = True
            self.set_desired_force(
                fd=[0, 0, -20, 0, 0, 0],
                dir=[0, 0, 1, 0, 0, 0],
                mod=self.DR_FC_MOD_REL,
            )
            force_started = True
            self.wait(0.5)

            for _ in range(3):
                self.movel(
                    self.posx(0, 0, 0, 0, 0, -10),
                    vel=50,
                    acc=50,
                    ref=self.DR_TOOL,
                    mod=self.DR_MV_MOD_REL,
                )
                self.wait(0.1)

            for attempt in range(1, max_attempts + 1):
                self.get_logger().info(
                    f"spin 뚜껑 회전 시도: {attempt}/{max_attempts}"
                )

                for _ in range(2):
                    self.movel(
                        self.posx(0, 0, 0, 0, 0, -90),
                        vel=25,
                        acc=25,
                        ref=self.DR_TOOL,
                        mod=self.DR_MV_MOD_REL,
                    )

                self.release()
                self.wait(0.3)

                for _ in range(2):
                    self.movel(
                        self.posx(0, 0, 0, 0, 0, 90),
                        vel=30,
                        acc=30,
                        ref=self.DR_TOOL,
                        mod=self.DR_MV_MOD_REL,
                    )
                    self.movel(
                        self.posx(0, 0, -1, 0, 0, 0),
                        vel=30,
                        acc=30,
                        ref=self.DR_TOOL,
                        mod=self.DR_MV_MOD_REL,
                    )

                self.movel(
                    self.posx(0, 0, 3, 0, 0, 0),
                    vel=25,
                    acc=25,
                    ref=self.DR_TOOL,
                    mod=self.DR_MV_MOD_REL,
                )
                self.grip()
                self.wait(0.5)

                if force_started:
                    self.release_force()
                    force_started = False
                if compliance_started:
                    self.release_compliance_ctrl()
                    compliance_started = False
                self.wait(0.5)

                self.movel(
                    self.posx(0, 0, -1, 0, 0, 0),
                    vel=15,
                    acc=15,
                    ref=self.DR_TOOL,
                    mod=self.DR_MV_MOD_REL,
                )
                current_force = self.get_tool_force(ref=self.DR_TOOL)

                if abs(float(current_force[2])) <= 3.0:
                    self.get_logger().info("spin 뚜껑 분리 성공")
                    self.movel(
                        self.posx(0, 0, -40, 0, 0, 0),
                        vel=20,
                        acc=20,
                        ref=self.DR_TOOL,
                        mod=self.DR_MV_MOD_REL,
                    )
                    return True

                self.get_logger().info(
                    "뚜껑 저항 감지. 다음 회전을 재시도함: "
                    f"Fz={abs(float(current_force[2])):.2f} N"
                )
                self.release()
                self.movel(
                    self.posx(0, 0, 1, 0, 0, 0),
                    vel=15,
                    acc=15,
                    ref=self.DR_TOOL,
                    mod=self.DR_MV_MOD_REL,
                )
                self.grip()

                self.task_compliance_ctrl(
                    stx=[10000, 10000, 300, 10000, 10000, 10000]
                )
                compliance_started = True
                self.set_desired_force(
                    fd=[0, 0, -20, 0, 0, 0],
                    dir=[0, 0, 1, 0, 0, 0],
                    mod=self.DR_FC_MOD_REL,
                )
                force_started = True
                self.wait(0.5)
                self.movel(
                    self.posx(0, 0, 5, 0, 0, 0),
                    vel=15,
                    acc=15,
                    ref=self.DR_TOOL,
                    mod=self.DR_MV_MOD_REL,
                )

            self.get_logger().error(
                f"spin 뚜껑 열기 최대 시도 초과: {max_attempts}회"
            )
            return False

        finally:
            if force_started:
                self.release_force()
            if compliance_started:
                self.release_compliance_ctrl()

    def check_grasp_success(self):
        try:
            self.wait_for_bottle_grip(
                timeout_sec=BOTTLE_GRIP_TIMEOUT_SEC
            )
            return True
        except GraspTimeoutError as error:
            self.get_logger().error(f"그리퍼 파지 확인 실패: {error}")
            return False

    def discard_lid(self):
        self.movejx(
            self.X_TRASH_DROP,
            vel=self.vel,
            acc=self.acc,
            ref=self.DR_BASE,
            sol=2,
        )
        self.release()
        self.get_logger().info("약통 뚜껑 버리기 완료")

    def spin_lid(self):
        self.get_logger().info("=== 돌려서 여는 약통 뚜껑 열기 ===")

        self.movejx(
            self.X_SPIN_LID_ABOVE,
            vel=self.vel,
            acc=self.acc,
            ref=self.DR_BASE,
            sol=2,
        )
        self.movel(
            self.posx(0, 0, -35, 0, 0, 0),
            vel=15,
            acc=15,
            ref=self.DR_BASE,
            mod=self.DR_MV_MOD_REL,
        )
        self.grip()

        if not self.check_grasp_success():
            self.release()
            self.movel(
                self.posx(0, 0, 50, 0, 0, 0),
                vel=20,
                acc=20,
                ref=self.DR_BASE,
                mod=self.DR_MV_MOD_REL,
            )
            return False

        if not self.spin_open():
            return False

        self.discard_lid()
        return True

    def run_opener_lid_task(self):
        self.get_logger().info("opener 약통 파지 및 뚜껑 개방 시작")

        self.storage_grasp()
        self.fix_lid()

        if self.lid_type == "pull":
            success = self.pull_lid()
        elif self.lid_type == "spin":
            success = self.spin_lid()
        else:
            raise ValueError(f"지원하지 않는 lid_type: {self.lid_type}")

        if not success:
            raise RuntimeError(
                f"뚜껑 개방 실패: medicine={self.medicine_name}, "
                f"lid_type={self.lid_type}"
            )

        self.get_logger().info("opener 뚜껑 개방 완료")

    # --------------------------------------------------
    # 파지 실패 시 다시 파지 시도
    # --------------------------------------------------

    def grip_bottle_until_success(self):
        """
        약통 파지가 확인될 때까지 계속 재시도한다.

        전체 재시도 횟수 제한은 없지만,
        한 번의 파지 확인에는 timeout을 적용한다.
        """
        attempt = 0

        self.release()
        sleep(0.5)  

        while rclpy.ok():
            attempt += 1

            self.get_logger().info(
                f"{self.medicine_name} 약통 파지 시도: "
                f"{attempt}회"
            )

            try:
                # 약통 보관 위치로 다시 이동
                self.movejx(self.X_LOCK_RETURN, vel=self.vel, acc=self.acc, sol=2)

                sleep(0.5)

                # 그리퍼 닫기
                self.grip()

                # 한 번의 파지 확인에는 timeout 사용
                self.wait_for_bottle_grip(
                    timeout_sec=BOTTLE_GRIP_TIMEOUT_SEC
                )

                self.get_logger().info(
                    f"{self.medicine_name} 약통 파지 성공: "
                    f"attempt={attempt}"
                )

                return

            except GraspTimeoutError as e:
                self.get_logger().warning(
                    f"{self.medicine_name} 약통 파지 실패: "
                    f"attempt={attempt}, "
                    f"error={e}"
                )

                # 실패한 상태로 그리퍼가 닫혀 있을 수 있으므로 다시 연다.
                try:
                    self.release()

                except Exception as release_error:
                    raise GraspError(
                        "파지 실패 후 그리퍼를 열 수 없어 "
                        "재시도를 중단함"
                    ) from release_error

                self.get_logger().info(
                    f"{self.medicine_name} 약통 파지를 다시 시도함"
                )

                sleep(0.5)

        raise GraspError(
            "ROS2 종료로 약통 파지 재시도가 중단됨"
        )
    # --------------------------------------------------
    # 7. 약통 집기 및 힘제어 상승
    # --------------------------------------------------
    def pick_medicine(self):
        if self.X_LOCK_RETURN is None:
            raise RuntimeError("약 보관 위치가 설정되지 않음")

        self.get_logger().info(
            f"{self.medicine_name} 약통 집기 시작"
        )

        self.grip_bottle_until_success()

        # 파지가 확인된 경우에만 약통 회전
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
            # task_compliance_ctrl()/set_desired_force()는 미리 설정한

            self.task_compliance_ctrl(stx=COMPLIANCE_STX)
            compliance_started = True
            self.wait(0.5)

            self.set_desired_force(
                fd=DESIRED_FORCE,
                dir=FORCE_DIRECTION,
                mod=self.DR_FC_MOD_REL,
            )
            force_started = True
            self.wait(0.5)
            

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
            if force_started:
                self.release_force()

            if compliance_started:
                self.release_compliance_ctrl()

        self.get_logger().info("약통 힘제어 상승 완료")

    # --------------------------------------------------
    # 7. 조제기로 이동
    # --------------------------------------------------
    def move_to_dispensing_position(self):
        if self.X_DISPENSER_POS is None:
            raise RuntimeError("조제기 시작 Joint 위치가 설정되지 않음")

        self.get_logger().info(
            f"{self.medicine_name} 약통을 들고 조제기 위치로 이동 시작"
        )

        self.movej(
            self.X_DISPENSER_POS,
            vel=self.vel,
            acc=self.acc,
        )

        self.get_logger().info(
            f"{self.medicine_name} 조제기 붓기 시작 위치 도착"
        )

    # 기존 외부 호출과의 호환을 위한 래퍼
    def move_pour(self):
        self.move_to_dispensing_position()
        self.pour_tweezer()

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

        self.movel(self.posx(0,-50,0,0,0,0), vel=20, acc=20, mod=self.DR_MV_MOD_REL)


    # --------------------------------------------------
    # 10. 약통 버리기
    # --------------------------------------------------
    def move_trash(self):
        if self.X_TRASH_DROP is None:
            raise RuntimeError("쓰레기통 위치가 설정되지 않음")

        self.get_logger().info("약통 버리기 시작")

        self.movejx(
            self.X_TRASH_DROP,
            vel=self.vel,
            acc=self.acc,
            ref=self.DR_BASE,
            sol=2,
        )

        self.release()

        self.get_logger().info("약통 버리기 완료")


    # --------------------------------------------------
    # 12. 서랍 닫기
    # --------------------------------------------------
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

    # --------------------------------------------------
    # 13. DB 리필 완료 알림
    # --------------------------------------------------
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
            "작업 시작"
        )

        self.open_drawer()
        self.run_opener_lid_task()
        self.pick_medicine()
        self.move_to_dispensing_position()
        self.pour_tweezer()
        self.move_trash()
        self.close_drawer()
        self.notify_done()
        self.get_logger().info("작업 완료")

    # --------------------------------------------------
    # 16. 전체 실행
    # --------------------------------------------------
    def run(self):
        self.get_logger().info("전체 작업 시작"
)

        self.wait_for_task()

        while rclpy.ok():
            while self.task_queue:
                task = self.task_queue.pop(0)
                task_key = self._make_task_key(task)
                if task_key is not None:
                    self.queued_task_keys.discard(task_key)
                    self.active_task_key = task_key

                try:
                    self.set_task_from_data(task)

                    self.get_logger().info(
                        f"작업 시작: "
                        f"{self.medicine_name}, "
                        f"lid_type={self.lid_type}, "
                        f"refill_amount={self.refill_amount}"
                    )

                    if self.lid_type == "pull":
                        self.run_gripper_lid_task()

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

                    if task_key is not None:
                        self.recently_completed_task_times[task_key] = (
                            time.monotonic()
                        )
                except GraspError as e:
                    self.get_logger().error(
                        "약통 파지 작업을 계속할 수 없어 전체 작업 중단: "
                        f"medicine={self.medicine_name}, "
                        f"error={e}"
                    )

                    try:
                        self.release()

                    except Exception as release_error:
                        self.get_logger().error(
                            f"작업 중단 후 그리퍼 열기 실패: "
                            f"{release_error}"
                        )

                    raise

                except Exception as e:
                    self.get_logger().error(
                        f"현재 작업 실행 실패: {e}"
                    )
                    raise

                finally:
                    self.active_task_key = None

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
            "pour_pills_opener",
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
            get_digital_input,
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
            get_tool_force,
            DR_BASE,
            DR_TOOL,
            DR_MV_MOD_REL,
            DR_FC_MOD_REL,
            wait
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
            "get_digital_input": get_digital_input,
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
            "get_tool_force": get_tool_force,
            "wait": wait
        }

        dsr_constants = {
            "DR_BASE": DR_BASE,
            "DR_TOOL": DR_TOOL,
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
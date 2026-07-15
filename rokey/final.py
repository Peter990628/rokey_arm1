# pharmacy_robot_combined_fifo.py
# 리필 로봇 + 스크래퍼/종이봉투 로봇 통합 노드
# 두 종류의 작업을 수신한 순서대로 하나의 FIFO 큐에서 실행한다.

import json
import math
import os
import time
from time import sleep

import rclpy
import DR_init
import requests
from std_msgs.msg import Bool, String


ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

TCP_NAME = "Tool_v1"
TOOL_NAME = "Tool Weight"

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

# 임시 설정: 실제 파지 DI 확인을 사용하지 않고 그리퍼 닫기 후 바로 진행
USE_BOTTLE_GRIP_SENSOR = False

# 같은 EventItem 작업의 반복 publish를 막는 시간
RECENT_TASK_IGNORE_SEC = 30.0

# 완료 직후 DDS 큐에 남아 있을 수 있는 동일 약의 오래된 메시지를 잠시 무시한다.
# 새 Event가 정말 REFILL_REQUIRED 상태라면 Task Manager가 계속 publish하므로,
# 이 시간이 지난 뒤 정상적으로 다시 작업에 등록된다.
RECENT_MEDICINE_IGNORE_SEC = 3.0


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
        self.recently_completed_medicine_times = {}

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
            357.12, 219.79, 450.52,
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
        """
        같은 polling 메시지는 중복으로 넣지 않되,
        같은 약이 새로운 EventItem으로 다시 들어오면 새 작업으로 구분한다.

        우선순위:
        1. source_event_item_ids
        2. source_event_ids + medicine_name
        3. medicine_name (이전 Task Manager 호환용)
        """
        if not isinstance(task, dict):
            return None

        medicine_name = task.get("medicine_name")
        if isinstance(medicine_name, str):
            medicine_name = medicine_name.strip()
        else:
            medicine_name = ""

        item_ids = task.get("source_event_item_ids")
        if isinstance(item_ids, list):
            normalized_item_ids = sorted(
                {
                    str(item_id)
                    for item_id in item_ids
                    if item_id is not None
                }
            )
            if normalized_item_ids:
                return (
                    f"medicine_name:{medicine_name}|"
                    f"event_items:{','.join(normalized_item_ids)}"
                )

        event_ids = task.get("source_event_ids")
        if isinstance(event_ids, list):
            normalized_event_ids = sorted(
                {
                    str(event_id)
                    for event_id in event_ids
                    if event_id is not None
                }
            )
            if normalized_event_ids:
                return (
                    f"medicine_name:{medicine_name}|"
                    f"events:{','.join(normalized_event_ids)}"
                )

        if medicine_name:
            return f"medicine_name:{medicine_name}"

        return None

    def _purge_old_completed_task_keys(self):
        now = time.monotonic()

        expired_task_keys = [
            key
            for key, completed_at in self.recently_completed_task_times.items()
            if now - completed_at >= RECENT_TASK_IGNORE_SEC
        ]
        for key in expired_task_keys:
            del self.recently_completed_task_times[key]

        expired_medicine_names = [
            medicine_name
            for medicine_name, completed_at
            in self.recently_completed_medicine_times.items()
            if now - completed_at >= RECENT_MEDICINE_IGNORE_SEC
        ]
        for medicine_name in expired_medicine_names:
            del self.recently_completed_medicine_times[medicine_name]

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

                medicine_name = task.get("medicine_name")
                if isinstance(medicine_name, str):
                    medicine_name = medicine_name.strip()
                else:
                    medicine_name = ""

                if (
                    task_key in self.queued_task_keys
                    or task_key == self.active_task_key
                    or task_key in self.recently_completed_task_times
                    or medicine_name in self.recently_completed_medicine_times
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

        self.set_tool("Tool Weight")
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

        USE_BOTTLE_GRIP_SENSOR가 False이면 DI 확인을 생략하고
        그리퍼 닫기 동작이 끝난 뒤 바로 성공으로 처리한다.
        """
        if not USE_BOTTLE_GRIP_SENSOR:
            self.get_logger().warning(
                "약통 파지 DI 확인 비활성화: 그리퍼 닫힘을 성공으로 간주함"
            )
            return

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
            self.posx(-110, 0, 0, 0, 0, 0),
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

        self.movel(
            self.posx(-30, 0, 0, 0, 0, 0),
            vel=20,
            acc=20,
            ref=self.DR_BASE,
            mod=self.DR_MV_MOD_REL,
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

        try:
            self.release_force()
        except Exception as e:
            self.get_logger().warning(
                f"힘제어 해제 생략: {e}"
            )

        try:
            self.release_compliance_ctrl()
        except Exception as e:
            self.get_logger().warning(
                f"순응제어 해제 생략: {e}"
            )

        self.wait(0.3)

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
        z_tolerance = 2.0
        start_time = time.monotonic()

        try:
            self.task_compliance_ctrl(
                stx=[500, 500, 100, 5000, 5000, 5000]
            )
            compliance_started = True
            self.wait(0.5)

            self.set_desired_force(
                fd=[0, 0, -45, 0, 0, 0],
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

                if time.monotonic() - start_time > 30.0:
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

        try:
            self.release_force()
        except Exception as e:
            self.get_logger().warning(
                f"힘제어 해제 생략: {e}"
            )

        try:
            self.release_compliance_ctrl()
        except Exception as e:
            self.get_logger().warning(
                f"순응제어 해제 생략: {e}"
            )

        self.wait(0.3)

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

        try:
            self.release_force()
        except Exception as e:
            self.get_logger().warning(
                f"힘제어 해제 생략: {e}"
            )

        try:
            self.release_compliance_ctrl()
        except Exception as e:
            self.get_logger().warning(
                f"순응제어 해제 생략: {e}"
            )

        self.wait(0.3)
        self.movel(
            self.posx(0, 0, -100, 0, 0, 0),
            vel=15,
            acc=15,
            ref=self.DR_BASE,
            mod=self.DR_MV_MOD_REL,
        )
        self.release()
        self.movel(
            self.posx(0, 0, 100, 0, 0, 0),
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

        # 오프너 작업에는 전용 opener_tweezer() 시퀀스를 사용한다.
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

        self.grip()

        self.movel(
            self.posx(110, 0, 0, 0, 0, 0),
            vel=10,
            acc=10,
            ref=self.DR_BASE,
            mod=self.DR_MV_MOD_REL,
        )

        self.release()

        self.movej(self.posj(0,0,90,0,90,0), vel =20, acc=20)

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

                    completed_at = time.monotonic()

                    if task_key is not None:
                        self.recently_completed_task_times[task_key] = (
                            completed_at
                        )

                    if self.medicine_name:
                        self.recently_completed_medicine_times[
                            self.medicine_name.strip()
                        ] = completed_at
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



# ==================================================
# 스크래퍼/종이봉투 작업 설정
# ==================================================
SCRAPER_TASK_TOPIC = "/dsr01/pharmacy/scraper_task"

PAPER_VELOCITY, PAPER_ACC = 30, 30
SCRAPER_VELOCITY, SCRAPER_ACC = 50, 50
SCRAPER_LINEAR_V, SCRAPER_LINEAR_A = 60, 60

TCP_GRIPPER_ONLY = "Tool_v1"
TCP_WITH_SCRAPER = "Tool_scraper"

HOME = [0, 0, 90, 0, 90, 0]

PAPER_BAG_ABOVE = [-17.36, 13.85, 88.76, -20.56, 71.08, 77.09]
PAPER_BAG_GRIP_MIDDLE = [-39.26, 40.82, 55.66, -2.99, 80.47, 48.32]
PAPER_BAG_GRIP = [-36.27, 47.72, 68.28, -3.37, 60.99, 49.45]

SHELF_X_MIN, SHELF_X_MAX = 210.0, 230.0
SHELF_Y_MIN, SHELF_Y_MAX = -230.0, -170.0
SHELF_Z_MIN, SHELF_Z_MAX = 238.0, 250.0

TOOL_STAND_SCRAPER = [15.82, 18.05, 97.40, -13.95, 29.76, 26.46]
DISPENSING_POINT = [-41.82, 10.51, 118.59, 72.71, 34.14, -99.73]
POUCH_POS_MIDDLE = [11.52, 8.37, 117.84, 119.58, -54.84, -69.72]
POUCH_POS_ABOVE = [10.31, 25.37, 85.48, 131.18, -83.59, -64.88]
POUCH_POS = [-11.49, 20.27, 77.08, 159.48, -76.69, -97.15]
SCRAPER_RETURN_MIDDLE = [14.69, -0.88, 91.28, 179.97, -89.60, -73.89]
SCRAPER_RETURN_MIDDLE_MIDDLE = [14.83, 4.70, 57.95, 179.95, -117.35, -73.88]
SCRAPER_RETURN_STAND = [16.36, 12.25, 99.06, 169.71, -36.23, -159.68]


class ScraperPaperBagWorker:
    def __init__(self, node):
        self.node = node
        self.task_done_pub = node.create_publisher(Bool, "task_done", 10)

        # Task Manager가 모든 EventItem이 READY인 Event를 배열로 발행한다.
        self.scraper_task_sub = node.create_subscription(
            String,
            SCRAPER_TASK_TOPIC,
            self.scraper_task_callback,
            1,
        )

        self.scraper_task_queue = []
        self.queued_event_ids = set()
        self.active_event_id = None

        # Event 상태를 PROCESSING/DONE으로 변경하지 않으므로, 같은 WAITING Event가
        # 반복 발행되어도 현재 프로세스에서 한 번만 실행되도록 기억한다.
        self.handled_event_ids = set()

        self.node.get_logger().info(
            f"스크래퍼 작업 토픽 구독 시작: {SCRAPER_TASK_TOPIC}"
        )

    # ------------------------------------------------------------
    # Task Manager 스크래퍼 작업 수신
    # ------------------------------------------------------------
    def scraper_task_callback(self, msg):
        try:
            tasks = json.loads(msg.data)

            if not isinstance(tasks, list):
                raise ValueError(
                    "스크래퍼 작업 데이터는 JSON 배열이어야 합니다."
                )

            added_count = 0
            skipped_count = 0

            for task in tasks:
                if not isinstance(task, dict):
                    skipped_count += 1
                    continue

                event_id = task.get("event_id")
                if event_id is None:
                    self.node.get_logger().warn(
                        f"event_id가 없는 스크래퍼 작업을 건너뜁니다: {task}"
                    )
                    skipped_count += 1
                    continue

                try:
                    event_id = int(event_id)
                except (TypeError, ValueError):
                    self.node.get_logger().warn(
                        f"event_id가 정수가 아니어서 건너뜁니다: {event_id}"
                    )
                    skipped_count += 1
                    continue

                # Task Manager가 READY 작업만 발행하지만, 잘못된 메시지로 인한
                # 오동작을 막기 위해 수신 노드에서도 한 번 더 확인한다.
                items = task.get("items", [])
                if not isinstance(items, list) or not items:
                    self.node.get_logger().warn(
                        f"items가 비어 있어 스크래퍼 작업을 건너뜁니다: "
                        f"event_id={event_id}"
                    )
                    skipped_count += 1
                    continue

                all_ready = all(
                    isinstance(item, dict)
                    and str(item.get("status", "")).strip().upper() == "READY"
                    for item in items
                )
                if not all_ready:
                    self.node.get_logger().warn(
                        f"READY가 아닌 항목이 있어 스크래퍼 작업을 건너뜁니다: "
                        f"event_id={event_id}"
                    )
                    skipped_count += 1
                    continue

                if (
                    event_id in self.queued_event_ids
                    or event_id == self.active_event_id
                    or event_id in self.handled_event_ids
                ):
                    skipped_count += 1
                    continue

                normalized_task = dict(task)
                normalized_task["event_id"] = event_id
                self.scraper_task_queue.append(normalized_task)
                self.queued_event_ids.add(event_id)
                added_count += 1

            self.node.get_logger().info(
                "스크래퍼 작업 수신: "
                f"추가={added_count}, 중복/부적합 생략={skipped_count}, "
                f"대기={len(self.scraper_task_queue)}"
            )

        except json.JSONDecodeError as error:
            self.node.get_logger().error(
                f"스크래퍼 작업 JSON 변환 실패: {error}"
            )
        except Exception as error:
            self.node.get_logger().error(
                f"스크래퍼 작업 수신 실패: {error}"
            )

    def run_task_loop(self):
        """Task Manager 작업을 기다렸다가 Event별로 전체 시퀀스를 한 번 실행한다."""
        self.node.get_logger().info(
            "Task Manager 연동 모드: 스크래퍼 작업 대기 중"
        )

        while rclpy.ok():
            if not self.scraper_task_queue:
                rclpy.spin_once(self.node, timeout_sec=0.2)
                continue

            task = self.scraper_task_queue.pop(0)
            event_id = task["event_id"]
            prescription_name = task.get("prescription_name", "")

            self.queued_event_ids.discard(event_id)
            self.active_event_id = event_id

            self.node.get_logger().info(
                "스크래퍼 작업 시작: "
                f"event_id={event_id}, prescription={prescription_name}, "
                f"items={len(task.get('items', []))}개"
            )

            try:
                success = self.run_full_sequence()
                if success:
                    self.node.get_logger().info(
                        f"스크래퍼 작업 완료: event_id={event_id}"
                    )
                else:
                    self.node.get_logger().error(
                        f"스크래퍼 작업 실패: event_id={event_id}"
                    )
            except Exception as error:
                self.node.get_logger().error(
                    "스크래퍼 작업 실행 중 오류: "
                    f"event_id={event_id}, error={error}"
                )
            finally:
                # Event 상태 변경 API를 사용하지 않으므로 성공/실패와 관계없이
                # 같은 Event를 자동으로 다시 실행하지 않는다. 재시도가 필요하면
                # 노드를 다시 실행하거나 이 ID를 수동으로 제거해야 한다.
                self.handled_event_ids.add(event_id)
                self.active_event_id = None

                # 로봇 동작 중 쌓인 최신 Task Manager 메시지를 처리한다.
                rclpy.spin_once(self.node, timeout_sec=0.1)

    # ------------------------------------------------------------
    # log_current_pos 디버그용 로그 
    # ------------------------------------------------------------

    def log_current_pos(self) :
        current_posx, _ = get_current_posx(ref=DR_BASE)
        current_posj = get_current_posj()
        x = current_posx[0]
        y = current_posx[1]
        z = current_posx[2]
        self.node.get_logger().info(f"좌표 확인: x={x:.2f}, y={y:.2f}, z={z:.2f}")
        self.node.get_logger().info(f"좌표 확인(joint): j1={current_posj[0]:.2f}, j2={current_posj[1]:.2f}, j3={current_posj[2]:.2f}, j4={current_posj[3]:.2f}, j5={current_posj[4]:.2f}, j6={current_posj[5]:.2f} \n")

    # ------------------------------------------------------------
    # gripper 동작
    # ------------------------------------------------------------
    def grip(self):
        self.node.get_logger().info("grip: digital output 0 1")
        set_digital_output(1, OFF)
        set_digital_output(2, OFF)
        set_digital_output(1, OFF)
        set_digital_output(2, ON)
        sleep(1)

    def release(self):
        # manipulation_node_scraper.py 의 release() 동작 그대로 사용
        self.node.get_logger().info("release: digital output 1 1")
        set_digital_output(1, OFF)
        set_digital_output(2, OFF)
        set_digital_output(1, ON)
        set_digital_output(2, ON)
        sleep(1)

    # ------------------------------------------------------------
    # 3단계: 종이봉투를 잡아서 수납대 위에 내려놓기
    # ------------------------------------------------------------
    def run_paper_bag_sequence(self) -> bool:
        self.node.get_logger().info("[PAPER 0/6] 종이봉투 작업 시작")

        set_tcp(TCP_GRIPPER_ONLY)
        self.release()

        self.node.get_logger().info("[PAPER 1/6] 홈으로 이동")
        movej(posj(*HOME), vel=PAPER_VELOCITY, acc=PAPER_ACC)

        self.log_current_pos()
        wait(0.5)

        
        self.node.get_logger().info("[PAPER 2/6] 봉투 위로 이동")
        movej(posj(*PAPER_BAG_ABOVE), vel=PAPER_VELOCITY, acc=PAPER_ACC)
        self.log_current_pos()
        wait(0.5)
        movej(posj(*PAPER_BAG_GRIP_MIDDLE), vel=PAPER_VELOCITY, acc=PAPER_ACC)
        wait(0.5)

        self.node.get_logger().info("[PAPER 3/6] 봉투 위치로 내려가서 잡기")
        movej(posj(*PAPER_BAG_GRIP), vel=PAPER_VELOCITY, acc=PAPER_ACC)
        self.log_current_pos()
        self.grip()

        self.node.get_logger().info("[PAPER 4/6] 봉투를 아래로 살짝 빼고 옆으로 가기")
        wait(0.5)
        movel(
            posx(0, 0, -100, 0, 0, 0),
            mod=DR_MV_MOD_REL,
            ref=DR_BASE,
            v=50,
            a=50,
        )
        self.log_current_pos()

        amovel(
            posx(0, -100, 0, 0, 0, 0),
            mod=DR_MV_MOD_REL,
            ref=DR_BASE,
            v=50,
            a=50,
        )


        movel(
            posx(-300, 0, 0, 0, 0, 0),
            mod=DR_MV_MOD_REL,
            ref=DR_BASE,
            v=50,
            a=50,
        )
        self.log_current_pos()


        self.node.get_logger().info("[PAPER 5/6] 봉투를 위로 들어 올리기")
        wait(0.5)
        movel(
            posx(0, 0, 500, 0, 0, 0),
            mod=DR_MV_MOD_REL,
            ref=DR_BASE,
            v=50,
            a=50,
        )
        self.log_current_pos()

        self.node.get_logger().info("[PAPER 6/6] 수납대 위로 이동")
        wait(0.5)
        movel(
            posx(0, 300, 0, 0, 0, 0),
            mod=DR_MV_MOD_REL,
            ref=DR_BASE,
            v=100,
            a=50,
        )
        self.log_current_pos()

        wait(0.5)
        movel(
            posx(0, 0, -60, 0, 0, 0),
            mod=DR_MV_MOD_REL,
            ref=DR_BASE,
            v=50,
            a=50,
        )

        wait(0.5)
        
        if not self._is_at_paper_bag_shelf():
            self.node.get_logger().error(
                "수납대 목표 범위를 벗어나 봉투를 놓지 않습니다."
            )
            return False

        self.release()
        self.node.get_logger().info("종이봉투 수납대 배치 완료")
        return True   
                
                  
    def _is_at_paper_bag_shelf(self) -> bool:
        current_posx, _ = get_current_posx(ref=DR_BASE)

        x = current_posx[0]
        y = current_posx[1]
        z = current_posx[2]

        self.node.get_logger().info(
            f"수납대 도착 좌표 확인: x={x:.2f}, y={y:.2f}, z={z:.2f}"
        )

        x_ok = SHELF_X_MIN <= x <= SHELF_X_MAX
        y_ok = SHELF_Y_MIN <= y <= SHELF_Y_MAX
        z_ok = SHELF_Z_MIN <= z <= SHELF_Z_MAX

        if not x_ok:
            self.node.get_logger().warn(
                f"수납대 X 범위 이탈: {x:.2f} "
                f"(허용 {SHELF_X_MIN}~{SHELF_X_MAX})"
            )

        if not y_ok:
            self.node.get_logger().warn(
                f"수납대 Y 범위 이탈: {y:.2f} "
                f"(허용 {SHELF_Y_MIN}~{SHELF_Y_MAX})"
            )

        if not z_ok:
            self.node.get_logger().warn(
                f"수납대 Z 범위 이탈: {z:.2f} "
                f"(허용 {SHELF_Z_MIN}~{SHELF_Z_MAX})"
            )

        return x_ok and y_ok and z_ok
    
    # ------------------------------------------------------------
    # 1단계: 스크래퍼를 집어서 조제기 배출구에서 대기
    # ------------------------------------------------------------
    def pickup_scraper_and_wait(self):
        self.node.get_logger().info("[SCRAPER 0/3] 스크래퍼 픽업 시작")

        set_tcp(TCP_GRIPPER_ONLY)

        self.node.get_logger().info("[SCRAPER 1/3] 홈으로 이동")
        movej(posj(*HOME), vel=SCRAPER_VELOCITY, acc=SCRAPER_ACC)
        self.log_current_pos()
        self.release()

        self.node.get_logger().info("[SCRAPER 2/3] 스크래퍼 거치대로 이동 후 잡기")
        movej(posj(*TOOL_STAND_SCRAPER), vel=SCRAPER_VELOCITY, acc=SCRAPER_ACC)
        self.log_current_pos()
        self.grip()

        set_tcp(TCP_WITH_SCRAPER)
        self.node.get_logger().info(f"TCP 전환: {TCP_WITH_SCRAPER}")

        self.node.get_logger().info("[SCRAPER 3/3] 조제기 배출구로 이동")

        movel(
            posx(0, 0, 160, 0, 0, 0),
            mod=DR_MV_MOD_REL,
            ref=DR_BASE,
            v=SCRAPER_LINEAR_V,
            a=SCRAPER_LINEAR_A,
        )
        self.log_current_pos()

        movel(
            posx(-150, 0, 0, 0, 0, 0),
            mod=DR_MV_MOD_REL,
            ref=DR_BASE,
            v=SCRAPER_LINEAR_V,
            a=SCRAPER_LINEAR_A,
        )
        self.log_current_pos()

        movel(
            posx(0, -400, 0, 0, 0, 0),
            mod=DR_MV_MOD_REL,
            ref=DR_BASE,
            v=SCRAPER_LINEAR_V,
            a=SCRAPER_LINEAR_A,
        )
        self.log_current_pos()

        movej(posj(*DISPENSING_POINT), vel=SCRAPER_VELOCITY, acc=SCRAPER_ACC)
        self.log_current_pos()

        movel(
            posx(-100, 0, 0, 0, 0, 0),
            mod=DR_MV_MOD_REL,
            ref=DR_BASE,
            v=SCRAPER_LINEAR_V,
            a=SCRAPER_LINEAR_A,
        )
        self.log_current_pos()

    # ------------------------------------------------------------
    # 2단계: 스크래퍼에 담긴 약을 봉투에 붓고 스크래퍼 반납
    # ------------------------------------------------------------
    def pour_and_return_scraper(self):
        movej(posj(*POUCH_POS_MIDDLE), vel=SCRAPER_VELOCITY, acc=SCRAPER_ACC)
        self.log_current_pos()
        self.node.get_logger().info("[POUR 1/3] 봉투 위치 위로 이동")
        movej(posj(*POUCH_POS_ABOVE), vel=SCRAPER_VELOCITY, acc=SCRAPER_ACC)
        self.log_current_pos()

        self.node.get_logger().info("[POUR 2/3] 봉투에 약 붓기")
        movej(posj(*POUCH_POS), vel=SCRAPER_VELOCITY, acc=SCRAPER_ACC)
        self.log_current_pos()
        sleep(0.5)

        self.node.get_logger().info("[POUR 3/3] 스크래퍼 거치대로 복귀 및 반납")
        movej(posj(*SCRAPER_RETURN_MIDDLE), vel=SCRAPER_VELOCITY, acc=SCRAPER_ACC)
        self.log_current_pos()
        sleep(0.5)

        movej(posj(*SCRAPER_RETURN_MIDDLE_MIDDLE), vel=SCRAPER_VELOCITY, acc=SCRAPER_ACC) 
        self.log_current_pos()
        sleep(0.5)

        movej(posj(*SCRAPER_RETURN_STAND), vel=SCRAPER_VELOCITY, acc=SCRAPER_ACC)
        self.log_current_pos()
        self.release() # 스크래퍼 놔주기

        # 거치대에 내려놓고 X 만큼 뒤로 빠진 후 home 으로 이동 할게요 해벽님~~~ 감사~~
        movel(
            posx(-100, 0, 0, 0, 0, 0),
            mod=DR_MV_MOD_REL,
            ref=DR_BASE,
            v=SCRAPER_LINEAR_V,
            a=SCRAPER_LINEAR_A,
        )
        self.log_current_pos()

        set_tcp(TCP_GRIPPER_ONLY)
        self.node.get_logger().info(f"TCP 전환: {TCP_GRIPPER_ONLY}")
        self.node.get_logger().info("스크래퍼 작업 종료.")
        movej(posj(*HOME), vel=SCRAPER_VELOCITY, acc=SCRAPER_ACC)
        self.log_current_pos()
        self.grip()

    # ------------------------------------------------------------
    # 전체 테스트 동작
    # ------------------------------------------------------------
    def run_full_sequence(self) -> bool:
        self.pickup_scraper_and_wait()
        self.pour_and_return_scraper()

        if not self.run_paper_bag_sequence():
            self.node.get_logger().warn("종이봉투 작업 실패로 전체 테스트를 중단합니다.")
            return False

        self.publish_task_done()
        return True

    def publish_task_done(self):
        done_msg = Bool()
        done_msg.data = True
        self.task_done_pub.publish(done_msg)
        self.node.get_logger().info("task_done published: True")
        rclpy.spin_once(self.node, timeout_sec=0.1)

# ==================================================
# 두 작업을 한 로봇에서 순차 실행하기 위한 Worker/Coordinator
# ==================================================
class RefillWorker(PourPills):
    """PourPills의 모션은 그대로 사용하고 독립 구독/실행 루프만 막는다."""

    def create_subscriptions(self):
        # 통합 Coordinator가 두 토픽을 모두 구독한다.
        self.sub_medicine = None

    def init_robot(self):
        # 통합 Coordinator 생성 후 로봇 초기화를 한 번만 수행한다.
        pass


class PharmacyRobotCoordinator:
    """
    리필 작업과 스크래퍼 작업을 하나의 FIFO 큐에 넣고 한 번에 하나씩 실행한다.

    큐 항목:
        ("refill", task)
        ("scraper", task)

    두 콜백이 실제로 처리된 순서대로 unified_task_queue에 append되므로,
    동시에 여러 작업이 대기하면 들어온 순서대로 실행된다.
    """

    def __init__(
        self,
        node,
        dsr_functions,
        dsr_constants,
        posx,
        posj,
    ):
        self.node = node
        self.unified_task_queue = []
        self.active_task_type = None

        # 기존 리필 로직을 모션 Worker로 사용한다.
        self.refill_worker = RefillWorker(
            node=node,
            dsr_functions=dsr_functions,
            dsr_constants=dsr_constants,
            posx=posx,
            posj=posj,
        )

        # 기존 스크래퍼 로직을 모션 Worker로 사용한다.
        # 생성자에서 만드는 기존 구독은 제거하고 Coordinator 구독만 사용한다.
        self.scraper_worker = ScraperPaperBagWorker(node)
        if self.scraper_worker.scraper_task_sub is not None:
            node.destroy_subscription(self.scraper_worker.scraper_task_sub)
            self.scraper_worker.scraper_task_sub = None

        self.refill_sub = node.create_subscription(
            String,
            MEDICINE_TOPIC,
            self.refill_task_callback,
            1,
        )
        self.scraper_sub = node.create_subscription(
            String,
            SCRAPER_TASK_TOPIC,
            self.scraper_task_callback,
            1,
        )

        # 로봇 초기화는 한 번만 한다.
        PourPills.init_robot(self.refill_worker)

        self.node.get_logger().info(
            "통합 작업 노드 준비 완료: "
            f"refill={MEDICINE_TOPIC}, scraper={SCRAPER_TASK_TOPIC}"
        )

    # --------------------------------------------------
    # 리필 토픽 수신 → 기존 중복 검사 후 통합 FIFO 큐로 이동
    # --------------------------------------------------
    def refill_task_callback(self, msg):
        before_count = len(self.refill_worker.task_queue)
        self.refill_worker.dispenser_pose_callback(msg)

        moved_count = 0
        while self.refill_worker.task_queue:
            task = self.refill_worker.task_queue.pop(0)
            self.unified_task_queue.append(("refill", task))
            moved_count += 1

        if moved_count > 0:
            self.node.get_logger().info(
                "통합 큐에 리필 작업 추가: "
                f"추가={moved_count}, 전체 대기={len(self.unified_task_queue)}"
            )

    # --------------------------------------------------
    # 스크래퍼 토픽 수신 → 기존 검증/중복 검사 후 통합 FIFO 큐로 이동
    # --------------------------------------------------
    def scraper_task_callback(self, msg):
        self.scraper_worker.scraper_task_callback(msg)

        moved_count = 0
        while self.scraper_worker.scraper_task_queue:
            task = self.scraper_worker.scraper_task_queue.pop(0)
            self.unified_task_queue.append(("scraper", task))
            moved_count += 1

        if moved_count > 0:
            self.node.get_logger().info(
                "통합 큐에 스크래퍼 작업 추가: "
                f"추가={moved_count}, 전체 대기={len(self.unified_task_queue)}"
            )

    # --------------------------------------------------
    # 리필 작업 1개 실행
    # --------------------------------------------------
    def execute_refill_task(self, task):
        worker = self.refill_worker
        task_key = worker._make_task_key(task)

        if task_key is not None:
            worker.queued_task_keys.discard(task_key)
            worker.active_task_key = task_key

        try:
            worker.set_task_from_data(task)
            self.node.get_logger().info(
                "[통합 큐] 리필 작업 시작: "
                f"medicine={worker.medicine_name}, "
                f"lid_type={worker.lid_type}, "
                f"refill_amount={worker.refill_amount}"
            )

            worker.run_gripper_lid_task()

            completed_at = time.monotonic()
            if task_key is not None:
                worker.recently_completed_task_times[task_key] = completed_at
            if worker.medicine_name:
                worker.recently_completed_medicine_times[
                    worker.medicine_name.strip()
                ] = completed_at

            self.node.get_logger().info(
                f"[통합 큐] 리필 작업 완료: {worker.medicine_name}"
            )
            return True

        except Exception as error:
            self.node.get_logger().error(
                "[통합 큐] 리필 작업 실패: "
                f"medicine={worker.medicine_name}, error={error}"
            )
            return False

        finally:
            worker.active_task_key = None

    # --------------------------------------------------
    # 스크래퍼 작업 1개 실행
    # --------------------------------------------------
    def execute_scraper_task(self, task):
        worker = self.scraper_worker
        event_id = int(task["event_id"])
        prescription_name = task.get("prescription_name", "")

        worker.queued_event_ids.discard(event_id)
        worker.active_event_id = event_id

        self.node.get_logger().info(
            "[통합 큐] 스크래퍼 작업 시작: "
            f"event_id={event_id}, prescription={prescription_name}, "
            f"items={len(task.get('items', []))}개"
        )

        try:
            success = worker.run_full_sequence()
            if success:
                self.node.get_logger().info(
                    f"[통합 큐] 스크래퍼 작업 완료: event_id={event_id}"
                )
            else:
                self.node.get_logger().error(
                    f"[통합 큐] 스크래퍼 작업 실패: event_id={event_id}"
                )
            return success

        except Exception as error:
            self.node.get_logger().error(
                "[통합 큐] 스크래퍼 작업 실행 중 오류: "
                f"event_id={event_id}, error={error}"
            )
            return False

        finally:
            # Event 상태 변경 API를 사용하지 않으므로 같은 event_id는
            # 현재 프로세스에서 다시 실행하지 않는다.
            worker.handled_event_ids.add(event_id)
            worker.active_event_id = None

    # --------------------------------------------------
    # 통합 FIFO 실행 루프
    # --------------------------------------------------
    def run(self):
        self.node.get_logger().info(
            "통합 FIFO 실행 시작. 리필/스크래퍼 작업 대기 중"
        )

        while rclpy.ok():
            if not self.unified_task_queue:
                rclpy.spin_once(self.node, timeout_sec=0.2)
                continue

            task_type, task = self.unified_task_queue.pop(0)
            self.active_task_type = task_type

            self.node.get_logger().info(
                "통합 큐 작업 꺼냄: "
                f"type={task_type}, 남은 작업={len(self.unified_task_queue)}"
            )

            try:
                if task_type == "refill":
                    self.execute_refill_task(task)
                elif task_type == "scraper":
                    self.execute_scraper_task(task)
                else:
                    self.node.get_logger().error(
                        f"알 수 없는 작업 종류: {task_type}"
                    )
            finally:
                self.active_task_type = None

            # 로봇 동작 중 들어온 토픽 콜백을 처리한다.
            rclpy.spin_once(self.node, timeout_sec=0.1)


def main(args=None):
    global movej, movel, amovel, movejx, set_tool, set_tcp
    global set_digital_output, get_digital_input
    global get_current_posx, get_current_posj
    global task_compliance_ctrl, set_desired_force
    global release_force, release_compliance_ctrl, get_tool_force
    global DR_BASE, DR_TOOL, DR_MV_MOD_REL, DR_FC_MOD_REL
    global posj, posx, wait

    rclpy.init(args=args)
    node = None

    try:
        node = rclpy.create_node(
            "pharmacy_robot_combined",
            namespace=ROBOT_ID,
        )
        DR_init.__dsr__node = node

        from DSR_ROBOT2 import (
            set_digital_output,
            get_digital_input,
            set_tool,
            set_tcp,
            movel,
            amovel,
            movej,
            movejx,
            task_compliance_ctrl,
            set_desired_force,
            get_current_posx,
            get_current_posj,
            release_force,
            release_compliance_ctrl,
            get_tool_force,
            DR_BASE,
            DR_TOOL,
            DR_MV_MOD_REL,
            DR_FC_MOD_REL,
            wait,
        )
        from DR_common2 import posx, posj

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
            "wait": wait,
        }

        dsr_constants = {
            "DR_BASE": DR_BASE,
            "DR_TOOL": DR_TOOL,
            "DR_MV_MOD_REL": DR_MV_MOD_REL,
            "DR_FC_MOD_REL": DR_FC_MOD_REL,
        }

        coordinator = PharmacyRobotCoordinator(
            node=node,
            dsr_functions=dsr_functions,
            dsr_constants=dsr_constants,
            posx=posx,
            posj=posj,
        )
        coordinator.run()

    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().info("Keyboard Interrupt")

    except ImportError as error:
        if node is not None:
            node.get_logger().error(
                f"DSR_ROBOT2 import 실패: {error}"
            )
        else:
            print(f"DSR_ROBOT2 import 실패: {error}")

    except Exception as error:
        if node is not None:
            node.get_logger().error(
                f"통합 로봇 노드 오류: {error}"
            )
        else:
            print(f"통합 로봇 초기화 오류: {error}")

    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# robot_total.py
# 약국 로봇 통합 제어 노드.
"""Pharmacy robot integrated control node.

This node subscribes to task_manager_bridge and combines the robot-control
roles of opener, pour_pills_rotate, and manipulator_test_2 while keeping the
tested motion sequences in their original modules.
"""
# ros2 run rokey robot_total --ros-args -p auto_start:=true
# 로봇 바로 시작

# ros2 topic pub --once /dsr01/pharmacy/dispensing_done std_msgs/msg/Bool "{data: true}"
# 스크래퍼 조제기 앞에 서면 실행 

#ros2 run rokey robot_total --ros-args -p wait_for_dispensing_done:=false
# 조제기 앞에서 대기 없이 실행 


from collections import deque
import json
import time
from urllib import error, request

import DR_init
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

import rokey.manipulator_test_2 as manipulator_module
import rokey.pour_pills_rotate as pour_module
from rokey.manipulator_test_2 import ManipulatorTest2
from rokey.opener import Opener
from rokey.pour_pills_rotate import (
    GraspError,
    GraspTimeoutError,
    PourPills,
)


ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

DEFAULT_BACKEND_BASE_URL = "http://172.23.0.128:8000/api"
DEFAULT_EVENTS_TOPIC = "/dsr01/pharmacy/events"
DEFAULT_MEDICINE_TOPIC = "/dsr01/pharmacy/medicine"
DEFAULT_REFILL_TOPIC = "/dsr01/pharmacy/refill_required_medicine"
DEFAULT_ENABLE_TOPIC = "/dsr01/pharmacy/robot_enable"
DEFAULT_DISPENSING_DONE_TOPIC = "/dsr01/pharmacy/dispensing_done"
DEFAULT_SCRAPER_READY_TOPIC = "/dsr01/pharmacy/scraper_ready"
DEFAULT_STATUS_TOPIC = "/dsr01/pharmacy/robot_status"

POSE_AXES = ("x", "y", "z", "rx", "ry", "rz")


DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


class IntegratedOpener(Opener):
    """Reuse opener motions without creating a second ROS node."""

    def __init__(
        self,
        node,
        dsr_functions,
        dsr_constants,
        posx,
        posj,
        force_timeout_sec,
    ):
        # Opener.__init__ creates its own Node and subscription, so the
        # integrated controller injects the single robot_total node instead.
        self.node = node

        for name, function in dsr_functions.items():
            setattr(self, name, function)
        for name, value in dsr_constants.items():
            setattr(self, name, value)

        self.posx = posx
        self.posj = posj
        self.vel = 60
        self.acc = 60
        self.force_timeout_sec = force_timeout_sec

        self.X_LOCK_RETURN = None
        self.X_OPENER_RETURN = None
        self.bottle_tip_offset_tcp = [0.0, 0.0, 75.0]
        self.medicine_name = None
        self.lid_type = None
        self.storage_loc = None

        self.define_positions()

    def configure_task(self, medicine):
        required = ["medicine_name", "lid_type"] + [
            f"storage_{axis}" for axis in POSE_AXES
        ]
        missing = [
            key
            for key in required
            if key not in medicine or medicine[key] is None
        ]
        if missing:
            raise KeyError(f"오프너 작업 데이터 누락: {', '.join(missing)}")

        self.medicine_name = str(medicine["medicine_name"])
        self.lid_type = str(medicine["lid_type"]).strip().lower()
        self.storage_loc = self.posx(
            *[float(medicine[f"storage_{axis}"]) for axis in POSE_AXES]
        )

    def open_bottle(self, medicine):
        self.configure_task(medicine)
        self.node.get_logger().info(
            f"[OPENER] {self.medicine_name} 개봉 시작: {self.lid_type}"
        )

        self.storage_grasp()
        self.fix_lid()

        if self.lid_type == "spin":
            # spin_lid() 안에서 뚜껑 폐기까지 한 번 실행된다.
            self.spin_lid()
        elif self.lid_type == "pull":
            self.pull_lid()
        else:
            raise ValueError(f"지원하지 않는 lid_type: {self.lid_type}")

        if self.X_LOCK_RETURN is None:
            raise RuntimeError("개봉 후 약통 고정 위치를 얻지 못함")

        self.node.get_logger().info(
            f"[OPENER] {self.medicine_name} 개봉 완료"
        )
        return self.X_LOCK_RETURN

    def pull_down(self):
        """Insert a bottle with force control and a finite timeout."""
        compliance_started = False
        force_started = False
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
            )
            force_started = True
            self.wait(0.5)

            target_z = 29.93
            z_tolerance = 1.0

            while rclpy.ok():
                current_z = self.get_current_pos_base()[2]
                if abs(current_z - target_z) <= z_tolerance:
                    self.node.get_logger().info(
                        f"거치대 안착 완료: z={current_z:.2f}"
                    )
                    return

                if time.monotonic() - start_time >= self.force_timeout_sec:
                    raise TimeoutError(
                        "거치대 삽입 시간 초과: "
                        f"current_z={current_z:.2f}, target_z={target_z:.2f}"
                    )
                time.sleep(0.02)
        finally:
            if force_started:
                self.release_force()
            if compliance_started:
                self.release_compliance_ctrl()

    def pull_lid_open(self):
        """Find the pull-type lid and fail instead of continuing on no contact."""
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

        for _ in range(100):
            current_force = self.get_tool_force(ref=self.DR_BASE)
            if current_force[0] > 0:
                self.node.get_logger().info(
                    f"PULL 뚜껑 접촉 감지: Fx={current_force[0]:.2f} N"
                )
                return

            self.movel(
                self.posx(-2.0, 0, 0, 0, 0, 0),
                vel=5,
                acc=5,
                ref=self.DR_BASE,
                mod=self.DR_MV_MOD_REL,
            )
            time.sleep(0.05)

        raise RuntimeError("PULL 방식 뚜껑 접촉 위치를 찾지 못함")

    def spin_open(self):
        """Run the existing ratchet motion and raise after five failures."""
        compliance_started = False
        force_started = False

        def start_pressing():
            nonlocal compliance_started, force_started
            self.task_compliance_ctrl(
                [10000, 10000, 300, 10000, 10000, 10000]
            )
            compliance_started = True
            self.set_desired_force(
                fd=[0, 0, -20, 0, 0, 0],
                dir=[0, 0, 1, 0, 0, 0],
            )
            force_started = True
            self.wait(0.5)

        def stop_pressing():
            nonlocal compliance_started, force_started
            if force_started:
                self.release_force()
                force_started = False
            if compliance_started:
                self.release_compliance_ctrl()
                compliance_started = False

        try:
            start_pressing()

            for _ in range(3):
                self.movel(
                    self.posx(0, 0, 0, 0, 0, -10),
                    vel=50,
                    acc=50,
                    ref=self.DR_TOOL,
                    mod=self.DR_MV_MOD_REL,
                )
                self.wait(0.1)

            max_attempts = 5
            for attempt in range(max_attempts):
                self.node.get_logger().info(
                    f"SPIN 뚜껑 회전 시도: {attempt + 1}/{max_attempts}"
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
                stop_pressing()

                self.movel(
                    self.posx(0, 0, -1, 0, 0, 0),
                    vel=15,
                    acc=15,
                    ref=self.DR_TOOL,
                    mod=self.DR_MV_MOD_REL,
                )
                current_force = self.get_tool_force(ref=self.DR_TOOL)

                if abs(current_force[2]) <= 3.0:
                    self.node.get_logger().info("SPIN 뚜껑 분리 확인 완료")
                    self.movel(
                        self.posx(0, 0, -40, 0, 0, 0),
                        vel=20,
                        acc=20,
                        ref=self.DR_TOOL,
                        mod=self.DR_MV_MOD_REL,
                    )
                    return

                self.node.get_logger().info(
                    f"SPIN 뚜껑 저항 감지: "
                    f"Fz={abs(current_force[2]):.2f} N"
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

                if attempt < max_attempts - 1:
                    start_pressing()
                    self.movel(
                        self.posx(0, 0, 5, 0, 0, 0),
                        vel=15,
                        acc=15,
                        ref=self.DR_TOOL,
                        mod=self.DR_MV_MOD_REL,
                    )
                    self.wait(0.5)

            raise RuntimeError("SPIN 뚜껑 개봉 5회 실패")
        finally:
            stop_pressing()


class IntegratedPourPills(PourPills):
    """Reuse pour motions with direct task and completion handoff."""

    def __init__(
        self,
        node,
        dsr_functions,
        dsr_constants,
        posx,
        posj,
        notify_refill,
        max_grip_attempts,
        lift_timeout_sec,
    ):
        self._notify_refill = notify_refill
        self._max_grip_attempts = max_grip_attempts
        self._lift_timeout_sec = lift_timeout_sec
        super().__init__(
            node=node,
            dsr_functions=dsr_functions,
            dsr_constants=dsr_constants,
            posx=posx,
            posj=posj,
        )

    def create_subscriptions(self):
        # robot_total supplies tasks directly from its internal queue.
        return

    def init_robot(self):
        # Tool/TCP initialization is owned by robot_total.
        return

    def grip_bottle_until_success(self):
        self.release()
        time.sleep(0.5)

        for attempt in range(1, self._max_grip_attempts + 1):
            self.get_logger().info(
                f"{self.medicine_name} 약통 파지 시도: "
                f"{attempt}/{self._max_grip_attempts}"
            )
            self.movejx(
                self.X_LOCK_RETURN,
                vel=self.vel,
                acc=self.acc,
                sol=2,
            )
            time.sleep(0.5)
            self.grip()

            try:
                self.wait_for_bottle_grip(
                    timeout_sec=pour_module.BOTTLE_GRIP_TIMEOUT_SEC
                )
                return
            except GraspTimeoutError:
                self.release()
                time.sleep(0.5)

        raise GraspError(
            f"{self.medicine_name} 약통 파지 "
            f"{self._max_grip_attempts}회 실패"
        )

    def lift_medicine_with_force(self):
        start_z = self.get_current_pos_base()[2]
        start_time = time.monotonic()
        compliance_started = False
        force_started = False

        try:
            self.task_compliance_ctrl(stx=pour_module.COMPLIANCE_STX)
            compliance_started = True
            self.wait(0.5)

            self.set_desired_force(
                fd=pour_module.DESIRED_FORCE,
                dir=pour_module.FORCE_DIRECTION,
                mod=self.DR_FC_MOD_REL,
            )
            force_started = True
            self.wait(0.5)

            while rclpy.ok():
                current_z = self.get_current_pos_base()[2]
                lifted = current_z - start_z
                if lifted >= pour_module.MEDICINE_LIFT_DISTANCE:
                    self.get_logger().info(
                        f"약통 힘제어 상승 완료: {lifted:.2f} mm"
                    )
                    return

                if time.monotonic() - start_time >= self._lift_timeout_sec:
                    raise TimeoutError(
                        "약통 힘제어 상승 시간 초과: "
                        f"lifted={lifted:.2f} mm"
                    )
                time.sleep(pour_module.FORCE_CHECK_INTERVAL_SEC)
        finally:
            if force_started:
                self.release_force()
            if compliance_started:
                self.release_compliance_ctrl()

    def notify_done(self):
        if not self._notify_refill(self.medicine_name, self.refill_amount):
            raise RuntimeError("백엔드 리필 완료 알림 실패")

    def run_after_open(self, medicine, lock_return_pose):
        self.set_task_from_data(medicine)
        self.X_LOCK_RETURN = lock_return_pose
        self.run_gripper_lid_task()


class RobotTotal(Node):
    def __init__(self):
        super().__init__("robot_total", namespace=ROBOT_ID)

        self.declare_parameter("backend_base_url", DEFAULT_BACKEND_BASE_URL)
        self.declare_parameter("request_timeout_sec", 2.0)
        self.declare_parameter("auto_start", False)
        self.declare_parameter("auto_package_waiting_events", True)
        self.declare_parameter("wait_for_dispensing_done", True)
        self.declare_parameter("dispensing_done_timeout_sec", 120.0)
        self.declare_parameter("force_motion_timeout_sec", 15.0)
        self.declare_parameter("max_grip_attempts", 3)

        self.backend_base_url = str(
            self.get_parameter("backend_base_url").value
        ).rstrip("/")
        self.request_timeout_sec = float(
            self.get_parameter("request_timeout_sec").value
        )
        self.robot_enabled = bool(self.get_parameter("auto_start").value)
        self.auto_package = bool(
            self.get_parameter("auto_package_waiting_events").value
        )
        self.wait_for_dispensing_done = bool(
            self.get_parameter("wait_for_dispensing_done").value
        )
        self.dispensing_done_timeout_sec = float(
            self.get_parameter("dispensing_done_timeout_sec").value
        )
        self.force_motion_timeout_sec = float(
            self.get_parameter("force_motion_timeout_sec").value
        )
        self.max_grip_attempts = int(
            self.get_parameter("max_grip_attempts").value
        )

        self.status_pub = self.create_publisher(
            String, DEFAULT_STATUS_TOPIC, 10
        )
        self.scraper_ready_pub = self.create_publisher(
            Bool, DEFAULT_SCRAPER_READY_TOPIC, 10
        )
        self.task_done_pub = None

        self.create_subscription(
            String,
            DEFAULT_EVENTS_TOPIC,
            self._events_callback,
            10,
        )
        self.create_subscription(
            String,
            DEFAULT_MEDICINE_TOPIC,
            self._medicine_callback,
            10,
        )
        self.create_subscription(
            String,
            DEFAULT_REFILL_TOPIC,
            self._refill_callback,
            10,
        )
        self.create_subscription(
            Bool,
            DEFAULT_ENABLE_TOPIC,
            self._enable_callback,
            10,
        )
        self.create_subscription(
            Bool,
            DEFAULT_DISPENSING_DONE_TOPIC,
            self._dispensing_done_callback,
            10,
        )

        self.refill_queue = deque()
        self.packaging_queue = deque()
        self.pending_refill_keys = set()
        self.completed_refill_keys = set()
        self.pending_event_ids = set()
        self.completed_event_ids = set()
        self.blocked_task_keys = set()

        self.busy = False
        self.robot_ready = False
        self.dispensing_done_received = False
        self.configured = False
        self.medicine_by_name = {}
        self.latest_refill_records = []
        self.missing_medicine_names = set()
        self.events_received = False
        self.refill_item_ids_by_name = {}

        self.dsr = None
        self.dsr_constants = None
        self.posx = None
        self.posj = None
        self.opener = None
        self.pour = None
        self.manipulator = None

        self.get_logger().info(
            "robot_total 생성 완료. task_manager_bridge 토픽 대기 중. "
            "로봇은 enable=True 신호 전까지 움직이지 않음"
        )

    def configure_robot(self, dsr_functions, dsr_constants, posx, posj):
        self.dsr = dsr_functions
        self.dsr_constants = dsr_constants
        self.posx = posx
        self.posj = posj

        self.opener = IntegratedOpener(
            node=self,
            dsr_functions=dsr_functions,
            dsr_constants=dsr_constants,
            posx=posx,
            posj=posj,
            force_timeout_sec=self.force_motion_timeout_sec,
        )
        self.pour = IntegratedPourPills(
            node=self,
            dsr_functions=dsr_functions,
            dsr_constants=dsr_constants,
            posx=posx,
            posj=posj,
            notify_refill=self._notify_refill_done,
            max_grip_attempts=self.max_grip_attempts,
            lift_timeout_sec=self.force_motion_timeout_sec,
        )

        # ManipulatorTest2 uses module-level DSR names, so bind them once to
        # the same DSR functions owned by robot_total.
        for name in (
            "movej",
            "movel",
            "amovel",
            "set_tool",
            "set_tcp",
            "set_digital_output",
            "get_current_posx",
            "get_current_posj",
            "wait",
        ):
            setattr(manipulator_module, name, dsr_functions[name])
        manipulator_module.DR_BASE = dsr_constants["DR_BASE"]
        manipulator_module.DR_MV_MOD_REL = dsr_constants["DR_MV_MOD_REL"]
        manipulator_module.posx = posx
        manipulator_module.posj = posj

        self.manipulator = ManipulatorTest2(self)
        # ManipulatorTest2 already creates this publisher on /dsr01/task_done.
        self.task_done_pub = self.manipulator.task_done_pub
        self.configured = True
        self._publish_status(
            "IDLE" if self.robot_enabled else "DISABLED",
            "DSR 초기화 완료",
        )

    def _enable_callback(self, msg):
        self.robot_enabled = bool(msg.data)
        state = "IDLE" if self.robot_enabled else "DISABLED"
        self.get_logger().info(f"로봇 enable 변경: {self.robot_enabled}")
        self._publish_status(state, "robot_enable 토픽 수신")

    def _dispensing_done_callback(self, msg):
        if msg.data:
            self.dispensing_done_received = True
            self.get_logger().info("조제 완료 신호 수신")

    def _events_callback(self, msg):
        try:
            events = self._records_from_message(msg, "events")
            refill_item_ids_by_name = {}
            for event in events:
                if not isinstance(event, dict):
                    continue
                items = event.get("items", [])
                if not isinstance(items, list):
                    continue
                for item in items:
                    if (
                        not isinstance(item, dict)
                        or item.get("status") == "READY"
                    ):
                        continue
                    medicine_name = item.get("medicine_name")
                    if medicine_name:
                        refill_item_ids_by_name.setdefault(
                            medicine_name, []
                        ).append(item.get("id"))

            self.events_received = True
            self.refill_item_ids_by_name = refill_item_ids_by_name
            self._drop_obsolete_refill_tasks()
            self._queue_latest_refills()

            if self.auto_package:
                self._queue_waiting_events(events)
        except (json.JSONDecodeError, ValueError) as exc:
            self.get_logger().error(f"events 토픽 처리 실패: {exc}")

    def _medicine_callback(self, msg):
        try:
            medicines = self._records_from_message(msg, "medicine")
            self.medicine_by_name = {
                medicine.get("medicine_name"): medicine
                for medicine in medicines
                if isinstance(medicine, dict)
                and medicine.get("medicine_name")
            }
            self.missing_medicine_names.clear()
            self._queue_latest_refills()
        except (json.JSONDecodeError, ValueError) as exc:
            self.get_logger().error(f"medicine 토픽 처리 실패: {exc}")

    def _refill_callback(self, msg):
        try:
            self.latest_refill_records = self._records_from_message(
                msg, "refill_required_medicine"
            )
            self._queue_latest_refills()
        except (json.JSONDecodeError, ValueError) as exc:
            self.get_logger().error(
                f"refill_required_medicine 토픽 처리 실패: {exc}"
            )

    def _records_from_message(self, msg, label):
        data = json.loads(msg.data)
        records = self._extract_records(data)
        if not isinstance(records, list):
            raise ValueError(f"{label} 데이터가 list 형식이 아님")
        return records

    def _queue_latest_refills(self):
        if not self.events_received:
            return

        for refill_record in self.latest_refill_records:
            if not isinstance(refill_record, dict):
                continue

            medicine_name = refill_record.get("medicine_name")
            if not medicine_name:
                continue
            if medicine_name not in self.refill_item_ids_by_name:
                continue

            medicine = self.medicine_by_name.get(medicine_name)
            if medicine is None:
                if medicine_name not in self.missing_medicine_names:
                    self.get_logger().warning(
                        "리필 토픽의 약을 medicine 토픽에서 아직 찾지 못함: "
                        f"{medicine_name}"
                    )
                    self.missing_medicine_names.add(medicine_name)
                continue

            # 리필 토픽의 최신 offset 값이 있으면 전체 medicine 정보에 덮어쓴다.
            task_medicine = dict(medicine)
            for axis in ("x", "y", "z"):
                key_name = f"bottle_tip_offset_{axis}"
                value = refill_record.get(key_name)
                if value is not None:
                    task_medicine[key_name] = value

            key = (
                "refill",
                medicine_name,
                tuple(
                    sorted(
                        str(item_id)
                        for item_id in self.refill_item_ids_by_name[
                            medicine_name
                        ]
                    )
                ),
                medicine.get("storage_stock"),
                medicine.get("dispensing_stock"),
            )
            known = (
                self.pending_refill_keys
                | self.completed_refill_keys
                | self.blocked_task_keys
            )
            if key in known:
                continue

            self.refill_queue.append(
                {
                    "kind": "refill",
                    "key": key,
                    "medicine": task_medicine,
                }
            )
            self.pending_refill_keys.add(key)
            self.get_logger().info(f"리필 작업 큐 추가: {medicine_name}")

    def _drop_obsolete_refill_tasks(self):
        retained = deque()
        for task in self.refill_queue:
            medicine_name = task["medicine"].get("medicine_name")
            if medicine_name in self.refill_item_ids_by_name:
                retained.append(task)
                continue

            self.pending_refill_keys.discard(task["key"])
            self.get_logger().info(
                f"더 이상 필요하지 않은 리필 작업 제거: {medicine_name}"
            )
        self.refill_queue = retained

    def _queue_waiting_events(self, events):
        waiting_events = [
            event
            for event in events
            if isinstance(event, dict) and event.get("status") == "WAITING"
        ]
        waiting_events.sort(key=lambda item: item.get("created_at", ""))

        for event in waiting_events:
            event_id = event.get("id")
            if event_id is None:
                continue
            key = ("packaging", event_id)
            if (
                event_id in self.pending_event_ids
                or event_id in self.completed_event_ids
                or key in self.blocked_task_keys
            ):
                continue

            self.packaging_queue.append(
                {"kind": "packaging", "key": key, "event": event}
            )
            self.pending_event_ids.add(event_id)
            self.get_logger().info(f"포장 작업 큐 추가: event_id={event_id}")

    def process_next_task(self):
        if not self.configured or not self.robot_enabled or self.busy:
            return

        if self.refill_queue:
            task = self.refill_queue.popleft()
        elif self.packaging_queue:
            task = self.packaging_queue.popleft()
        else:
            return

        self.busy = True
        key = task["key"]

        try:
            self._ensure_robot_ready()
            if task["kind"] == "refill":
                self._execute_refill(task["medicine"])
                self.pending_refill_keys.discard(key)
                self.completed_refill_keys.add(key)
            else:
                event_id = task["event"].get("id")
                self._execute_packaging(task["event"])
                self.pending_event_ids.discard(event_id)
                self.completed_event_ids.add(event_id)

            self._publish_status("IDLE", "작업 완료")
        except Exception as exc:
            self.get_logger().error(f"통합 작업 실패: {exc}")
            self.blocked_task_keys.add(key)
            if task["kind"] == "refill":
                self.pending_refill_keys.discard(key)
            else:
                self.pending_event_ids.discard(task["event"].get("id"))

            # 부분 동작 후 자동 재시작을 막기 위해 명시적 재활성화를 요구한다.
            self.robot_enabled = False
            self._publish_status("ERROR", str(exc))
        finally:
            self.busy = False

    def _ensure_robot_ready(self):
        if self.robot_ready:
            return

        self.dsr["set_tool"]("Tool Weight_1")
        self.dsr["set_tcp"]("Tool_v1")
        self.opener.release()
        self.dsr["movej"](
            self.opener.J_READY,
            vel=self.opener.vel,
            acc=self.opener.acc,
        )
        self.robot_ready = True
        self._publish_status("IDLE", "로봇 Ready 자세 도착")

    def _execute_refill(self, medicine):
        medicine_name = medicine.get("medicine_name", "unknown")
        self._publish_status("OPENING", f"{medicine_name} 뚜껑 개봉")
        self.dsr["set_tool"]("Tool Weight_1")
        self.dsr["set_tcp"]("Tool_v1")

        lock_return_pose = self.opener.open_bottle(medicine)

        self._publish_status("REFILLING", f"{medicine_name} 조제기 리필")
        self.pour.run_after_open(medicine, lock_return_pose)

    def _execute_packaging(self, event):
        event_id = event.get("id")
        self._publish_status(
            "SCRAPER_PICKUP", f"event_id={event_id} 스크래퍼 준비"
        )

        self.dispensing_done_received = False
        self.manipulator.pickup_scraper_and_wait()
        self._publish_bool(self.scraper_ready_pub, True)

        try:
            if self.wait_for_dispensing_done:
                self._publish_status(
                    "WAITING_DISPENSING",
                    f"event_id={event_id} 조제 완료 신호 대기",
                )
                deadline = (
                    time.monotonic() + self.dispensing_done_timeout_sec
                )
                while rclpy.ok() and not self.dispensing_done_received:
                    if not self.robot_enabled:
                        raise RuntimeError("조제 대기 중 robot_enable=False 수신")
                    if time.monotonic() >= deadline:
                        raise TimeoutError("조제 완료 신호 대기 시간 초과")
                    rclpy.spin_once(self, timeout_sec=0.1)
        finally:
            self._publish_bool(self.scraper_ready_pub, False)

        self._publish_status("PACKAGING", f"event_id={event_id} 포장 시작")
        self.manipulator.pour_and_return_scraper()
        if not self.manipulator.run_paper_bag_sequence():
            raise RuntimeError("종이봉투 수납대 배치 실패")

        self._publish_bool(self.task_done_pub, True)
        self.get_logger().info(f"처방 작업 완료: event_id={event_id}")

    def _notify_refill_done(self, medicine_name, amount):
        status_code, data = self._post_json(
            "/tasks/refill/",
            {"medicine_name": medicine_name, "amount": amount},
        )
        if status_code != 200:
            self.get_logger().error(
                f"리필 완료 POST 실패: {status_code} {data}"
            )
            return False

        self.get_logger().info(f"리필 완료 POST 성공: {data}")
        return True

    def _post_json(self, path, payload):
        url = f"{self.backend_base_url}{path}"
        body = json.dumps(payload).encode("utf-8")
        headers = {
            "Accept": "application/json",
            "Content-Type": "application/json",
        }

        req = request.Request(
            url,
            data=body,
            headers=headers,
            method="POST",
        )

        try:
            with request.urlopen(
                req, timeout=self.request_timeout_sec
            ) as response:
                raw = response.read().decode("utf-8")
                return response.status, self._decode_json(raw)
        except error.HTTPError as exc:
            raw = exc.read().decode("utf-8")
            return exc.code, self._decode_json(raw)
        except (error.URLError, TimeoutError) as exc:
            return 0, {"error": str(exc)}

    @staticmethod
    def _decode_json(raw):
        if not raw:
            return {}
        try:
            return json.loads(raw)
        except json.JSONDecodeError:
            return {"raw": raw}

    @staticmethod
    def _extract_records(data):
        if isinstance(data, list):
            return data
        if isinstance(data, dict) and isinstance(data.get("results"), list):
            return data["results"]
        return []

    @staticmethod
    def _publish_bool(publisher, value):
        msg = Bool()
        msg.data = bool(value)
        publisher.publish(msg)

    @staticmethod
    def _publish_json(publisher, data):
        msg = String()
        msg.data = json.dumps(data, ensure_ascii=False)
        publisher.publish(msg)

    def _publish_status(self, state, detail):
        self._publish_json(
            self.status_pub,
            {
                "state": state,
                "detail": detail,
                "enabled": self.robot_enabled,
            },
        )

    def run_loop(self):
        # DSR_ROBOT2 internally calls spin_until_future_complete(). Keeping
        # robot commands and ROS callbacks on this one loop avoids two
        # executors trying to spin the same node.
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            self.process_next_task()


def main(args=None):
    rclpy.init(args=args)
    node = RobotTotal()
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            DR_AXIS_Z,
            DR_BASE,
            DR_FC_MOD_REL,
            DR_MV_MOD_ABS,
            DR_MV_MOD_REL,
            DR_SSTOP,
            DR_TOOL,
            amove_periodic,
            amovel,
            check_position_condition,
            get_current_posj,
            get_current_posx,
            get_digital_input,
            get_tool_force,
            movej,
            movejx,
            movel,
            release_compliance_ctrl,
            release_force,
            set_desired_force,
            set_digital_output,
            set_tcp,
            set_tool,
            task_compliance_ctrl,
            trans,
            wait,
        )
        from DR_common2 import posj, posx

        dsr_functions = {
            "movej": movej,
            "movejx": movejx,
            "movel": movel,
            "amovel": amovel,
            "set_tool": set_tool,
            "set_tcp": set_tcp,
            "set_digital_output": set_digital_output,
            "get_digital_input": get_digital_input,
            "get_current_posx": get_current_posx,
            "get_current_posj": get_current_posj,
            "task_compliance_ctrl": task_compliance_ctrl,
            "set_desired_force": set_desired_force,
            "get_tool_force": get_tool_force,
            "release_force": release_force,
            "release_compliance_ctrl": release_compliance_ctrl,
            "trans": trans,
            "amove_periodic": amove_periodic,
            "check_position_condition": check_position_condition,
            "wait": wait,
        }
        dsr_constants = {
            "DR_BASE": DR_BASE,
            "DR_TOOL": DR_TOOL,
            "DR_MV_MOD_ABS": DR_MV_MOD_ABS,
            "DR_MV_MOD_REL": DR_MV_MOD_REL,
            "DR_FC_MOD_REL": DR_FC_MOD_REL,
            "DR_AXIS_Z": DR_AXIS_Z,
            "DR_SSTOP": DR_SSTOP,
        }

        node.configure_robot(
            dsr_functions=dsr_functions,
            dsr_constants=dsr_constants,
            posx=posx,
            posj=posj,
        )
        node.run_loop()
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt")
    except Exception as exc:
        node.get_logger().error(f"robot_total 실행 오류: {exc}")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

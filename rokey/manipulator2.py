# manipulator_test_2.py
# paper_bag_test + manipulation_node_scraper
# 담당자 : hyj, peter




import json
import sys
from time import sleep

import rclpy
import DR_init
from std_msgs.msg import Bool, String


ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

SCRAPER_TASK_TOPIC = "/dsr01/pharmacy/scraper_task"

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

ON, OFF = 1, 0

# ---- 속도 ----
PAPER_VELOCITY, PAPER_ACC = 30, 30
SCRAPER_VELOCITY, SCRAPER_ACC = 50, 50
SCRAPER_LINEAR_V, SCRAPER_LINEAR_A = 60, 60

# ---- TCP 프로필 ----
TCP_GRIPPER_ONLY = "Tool_v1"
TCP_WITH_SCRAPER = "Tool_scraper"

# ---- 공통 위치 ----
HOME = [0, 0, 90, 0, 90, 0]

# ---- 종이봉투 위치 ----
PAPER_BAG_ABOVE = [-17.36,13.85,88.76,-20.56,71.08,77.09]
PAPER_BAG_GRIP_MIDDLE = [-39.26, 40.82, 55.66, -2.99, 80.47, 48.32]
PAPER_BAG_GRIP = [-36.27, 47.72, 68.28, -3.37, 60.99, 49.45]

# ---- 수납대 최종 TCP 허용 범위 [mm] ----
SHELF_X_MIN, SHELF_X_MAX = 210.0, 230.0
SHELF_Y_MIN, SHELF_Y_MAX = -230.0, -170.0
SHELF_Z_MIN, SHELF_Z_MAX = 238.0, 250.0


# ---- 스크래퍼 위치 ----
TOOL_STAND_SCRAPER = [15.82, 18.05, 97.40, -13.95, 29.76, 26.46]
DISPENSING_POINT = [-41.82, 10.51, 118.59, 72.71, 34.14, -99.73]
POUCH_POS_MIDDLE =[11.52, 8.37, 117.84, 119.58, -54.84, -69.72]
POUCH_POS_ABOVE = [10.31, 25.37, 85.48, 131.18, -83.59, -64.88]
POUCH_POS = [-11.49, 20.27, 77.08, 159.48, -76.69, -97.15]
SCRAPER_RETURN_MIDDLE = [14.69, -0.88, 91.28, 179.97, -89.60, -73.89]
SCRAPER_RETURN_MIDDLE_MIDDLE = [14.83, 4.70, 57.95, 179.95, -117.35, -73.88]
SCRAPER_RETURN_STAND = [16.36, 12.25, 99.06, 169.71, -36.23, -159.68] 



class ManipulatorTest2:
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
        self.node.get_logger().info("manipulator_test_2 task_done published: True")
        rclpy.spin_once(self.node, timeout_sec=0.1)


def main(args=None):
    global movej, movel, amovel, set_tool, set_tcp
    global set_digital_output, get_current_posx, get_current_posj
    global DR_BASE, DR_MV_MOD_REL
    global posj, posx
    global wait

    rclpy.init(args=args)
    node = rclpy.create_node("manipulator_test_2", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            movej,
            movel,
            amovel,
            set_tool,
            set_tcp,
            set_digital_output,
            get_current_posx,
            get_current_posj,
            DR_BASE,
            DR_MV_MOD_REL,
            wait,
        )
        from DR_common2 import posj, posx
    except ImportError as e:
        node.get_logger().error(f"Error importing DSR_ROBOT2: {e}")
        node.destroy_node()
        rclpy.shutdown()
        return

    try:
        set_tool("Tool Weight_1")
        set_tcp(TCP_GRIPPER_ONLY)

        manipulator = ManipulatorTest2(node)

        if "--paper-only" in sys.argv:
            if manipulator.run_paper_bag_sequence():
                manipulator.publish_task_done()
        elif "--scraper-only" in sys.argv:
            manipulator.pickup_scraper_and_wait()
            manipulator.pour_and_return_scraper()
            manipulator.publish_task_done()
        elif "--run-once" in sys.argv:
            # Task Manager 없이 기존 전체 모션만 한 번 실행하는 테스트 옵션
            manipulator.run_full_sequence()
        else:
            # 기본 실행은 Task Manager의 /dsr01/pharmacy/scraper_task를 기다린다.
            manipulator.run_task_loop()

    except Exception as e:
        node.get_logger().error(f"manipulator_test_2 실행 중 오류: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
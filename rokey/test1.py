import math
from time import sleep

import rclpy
import DR_init
from visualization_msgs.msg import Marker


ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

# 버추얼 로봇에서 실제로 등록되어 있는 기존 TCP 이름
TCP_NAME = "Tool_v1"
TOOL_NAME = "Tool Weight_1"

# 테스트 시작 자세: DR_BASE 기준 posx [mm, degree]
TEST_START_JOINT = [
    -15.23,
    23.84,
    48.29,
    14.40,
    55.66,
    -108.35,
]

# 현재 활성 TCP 원점에서 가상 약통 끝점까지의 로컬 오프셋 [mm]
# TCP +Y 22.5 mm, TCP +Z 25.0 mm
# 거치대 끝 [0.0, 29, 10]
BOTTLE_TIP_OFFSET_TCP = [0.0, 25, -42]

# 우선 -10도로 확인한 뒤 -80도로 바꾸는 것을 권장
TEST_ROTATION_DEG = +60.0
ROTATION_STEP_DEG = 10.0

MOVE_VEL = 10
MOVE_ACC = 10
ROTATE_VEL = 15
ROTATE_ACC = 15
HOLD_SEC = 1.0

# RViz Marker가 표시될 기준 프레임
# 현재 DSR RViz의 base frame 이름과 다르면 수정
MARKER_FRAME_ID = "base_0"
MARKER_TOPIC = "/dsr01/virtual_bottle_tip"


DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


class VirtualBottleTipTest:
    def __init__(
        self,
        node,
        set_tool,
        set_tcp,
        movel,
        movej,
        get_current_posx,
        posx,
        posj,
        dr_base,
        
    ):
        self.node = node
        self.set_tool = set_tool
        self.set_tcp = set_tcp
        self.movel = movel
        self.movej = movej
        self.get_current_posx = get_current_posx
        self.posx = posx
        self.posj = posj
        self.DR_BASE = dr_base
        

        self.marker_pub = self.node.create_publisher(
            Marker,
            MARKER_TOPIC,
            10,
        )

    # --------------------------------------------------
    # 기본 행렬 계산
    # --------------------------------------------------
    @staticmethod
    def matmul(left, right):
        return [
            [
                sum(left[row][k] * right[k][col] for k in range(3))
                for col in range(3)
            ]
            for row in range(3)
        ]

    @staticmethod
    def matvec(matrix, vector):
        return [
            sum(matrix[row][col] * vector[col] for col in range(3))
            for row in range(3)
        ]

    @staticmethod
    def rotation_x(angle_rad):
        c = math.cos(angle_rad)
        s = math.sin(angle_rad)
        return [
            [1.0, 0.0, 0.0],
            [0.0, c, -s],
            [0.0, s, c],
        ]

    @staticmethod
    def rotation_y(angle_rad):
        c = math.cos(angle_rad)
        s = math.sin(angle_rad)
        return [
            [c, 0.0, s],
            [0.0, 1.0, 0.0],
            [-s, 0.0, c],
        ]

    @staticmethod
    def rotation_z(angle_rad):
        c = math.cos(angle_rad)
        s = math.sin(angle_rad)
        return [
            [c, -s, 0.0],
            [s, c, 0.0],
            [0.0, 0.0, 1.0],
        ]

    @classmethod
    def zyz_to_rotation_matrix(cls, a_deg, b_deg, c_deg):
        rz_a = cls.rotation_z(math.radians(a_deg))
        ry_b = cls.rotation_y(math.radians(b_deg))
        rz_c = cls.rotation_z(math.radians(c_deg))
        return cls.matmul(cls.matmul(rz_a, ry_b), rz_c)

    @staticmethod
    def angle_near_reference(angle_deg, reference_deg):
        return angle_deg + 360.0 * round(
            (reference_deg - angle_deg) / 360.0
        )

    @classmethod
    def rotation_matrix_to_zyz(cls, rotation, reference_abc):
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

        results = []

        for candidate in (candidate_1, candidate_2):
            adjusted = [
                cls.angle_near_reference(
                    candidate[index],
                    reference_abc[index],
                )
                for index in range(3)
            ]

            score = sum(
                (adjusted[index] - reference_abc[index]) ** 2
                for index in range(3)
            )
            results.append((score, adjusted))

        return min(results, key=lambda item: item[0])[1]

    @staticmethod
    def make_angle_sequence(total_angle_deg, step_deg):
        if step_deg <= 0.0:
            raise ValueError("ROTATION_STEP_DEG는 0보다 커야 합니다.")

        if abs(total_angle_deg) < 1e-9:
            return []

        direction = 1.0 if total_angle_deg > 0.0 else -1.0
        signed_step = abs(step_deg) * direction

        angles = []
        current = signed_step

        while abs(current) < abs(total_angle_deg):
            angles.append(current)
            current += signed_step

        angles.append(total_angle_deg)
        return angles

    # --------------------------------------------------
    # 현재 위치 읽기
    # --------------------------------------------------
    def get_current_pose(self):
        result = self.get_current_posx(
            ref=self.DR_BASE,
            
        )

        if result is None:
            raise RuntimeError("get_current_posx() 결과가 None입니다.")

        # 버전에 따라 posx 또는 (posx, sol) 형태 처리
        if (
            hasattr(result, "__len__")
            and len(result) >= 6
            and isinstance(result[0], (int, float))
        ):
            pose = result
        elif hasattr(result, "__len__") and len(result) >= 1:
            pose = result[0]
        else:
            raise RuntimeError(f"현재 자세 형식 오류: {result}")

        if pose is None or len(pose) < 6:
            raise RuntimeError(f"현재 자세 데이터 오류: {result}")

        return [float(value) for value in pose[:6]]

    # --------------------------------------------------
    # 가상 약통 끝점 Marker
    # --------------------------------------------------
    def publish_tip_marker(self, tip_position_base):
        marker = Marker()
        marker.header.frame_id = MARKER_FRAME_ID
        marker.header.stamp = self.node.get_clock().now().to_msg()

        marker.ns = "virtual_bottle_tip"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        # 로봇 좌표 mm -> ROS Marker m
        marker.pose.position.x = tip_position_base[0] / 1000.0
        marker.pose.position.y = tip_position_base[1] / 1000.0
        marker.pose.position.z = tip_position_base[2] / 1000.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.025
        marker.scale.y = 0.025
        marker.scale.z = 0.025

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # 노드 종료 전까지 계속 표시
        marker.lifetime.sec = 0

        self.marker_pub.publish(marker)

        self.node.get_logger().info(
            f"가상 약통 끝 Marker 발행: {MARKER_TOPIC}, "
            f"BASE(mm)={[round(v, 3) for v in tip_position_base]}"
        )

    # --------------------------------------------------
    # 약통 끝점을 고정한 회전 자세 계산
    # --------------------------------------------------
    def calculate_target_pose(
        self,
        start_rotation,
        tip_position_base,
        angle_deg,
        reference_abc,
    ):
        local_x_rotation = self.rotation_x(
            math.radians(angle_deg)
        )

        target_rotation = self.matmul(
            start_rotation,
            local_x_rotation,
        )

        target_tip_offset_base = self.matvec(
            target_rotation,
            BOTTLE_TIP_OFFSET_TCP,
        )

        target_position = [
            tip_position_base[index]
            - target_tip_offset_base[index]
            for index in range(3)
        ]

        target_abc = self.rotation_matrix_to_zyz(
            target_rotation,
            reference_abc,
        )

        target_values = target_position + target_abc

        target_pose = self.posx(
            *target_values,
           
        )

        return target_pose, target_values, target_abc

    # --------------------------------------------------
    # 단독 테스트 실행
    # --------------------------------------------------
    def run(self):
        self.node.get_logger().info("가상 약통 끝 회전 테스트 시작")

        # self.set_tool(TOOL_NAME)
        # self.set_tcp(Tool_v1)

        start_target = self.posj(
            *TEST_START_JOINT,
            
        )

        self.node.get_logger().info(
            f"테스트 시작 자세로 이동: {TEST_START_JOINT}"
        )

        # posx 목표이므로 movejx 사용
        self.movej(
            start_target,
            vel=MOVE_VEL,
            acc=MOVE_ACC,
        )

        sleep(1.0)

        # 실제 도달 자세를 다시 읽고 이를 회전 시작점으로 사용
        start_pose = self.get_current_pose()
        start_position = start_pose[:3]
        start_abc = start_pose[3:6]

        start_rotation = self.zyz_to_rotation_matrix(
            *start_abc
        )

        tip_offset_base = self.matvec(
            start_rotation,
            BOTTLE_TIP_OFFSET_TCP,
        )

        tip_position_base = [
            start_position[index] + tip_offset_base[index]
            for index in range(3)
        ]

        self.publish_tip_marker(tip_position_base)

        self.node.get_logger().info(
            "회전 기준 계산 완료: "
            f"start={[round(v, 3) for v in start_pose]}, "
            f"tip={[round(v, 3) for v in tip_position_base]}"
        )

        # Marker가 RViz에 표시될 시간
        for _ in range(10):
            rclpy.spin_once(self.node, timeout_sec=0.05)

        angles = self.make_angle_sequence(
            TEST_ROTATION_DEG,
            ROTATION_STEP_DEG,
        )

        calculated_poses = []
        reference_abc = start_abc

        for angle_deg in angles:
            target_pose, values, reference_abc = (
                self.calculate_target_pose(
                    start_rotation=start_rotation,
                    tip_position_base=tip_position_base,
                    angle_deg=angle_deg,
                    reference_abc=reference_abc,
                )
            )

            calculated_poses.append(target_pose)

            self.node.get_logger().info(
                f"회전 {angle_deg:.1f}도 목표: "
                f"{[round(v, 3) for v in values]}"
            )

            self.movel(
                target_pose,
                vel=ROTATE_VEL,
                acc=ROTATE_ACC,
                ref=self.DR_BASE,
            )

            self.publish_tip_marker(tip_position_base)
            rclpy.spin_once(self.node, timeout_sec=0.01)

        sleep(HOLD_SEC)

        self.node.get_logger().info("동일 경로로 시작 자세 복귀")

        for target_pose in reversed(calculated_poses[:-1]):
            self.movel(
                target_pose,
                vel=ROTATE_VEL,
                acc=ROTATE_ACC,
                ref=self.DR_BASE,
            )

        self.movel(
            self.posx(
                *start_pose,
               
            ),
            vel=ROTATE_VEL,
            acc=ROTATE_ACC,
            ref=self.DR_BASE,
        )

        self.publish_tip_marker(tip_position_base)

        self.node.get_logger().info(
            "가상 약통 끝 회전 테스트 완료"
        )

        # RViz에서 Marker를 확인할 수 있도록 잠시 유지
        for _ in range(40):
            rclpy.spin_once(self.node, timeout_sec=0.05)


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = rclpy.create_node(
            "virtual_bottle_tip_test",
            namespace=ROBOT_ID,
        )

        DR_init.__dsr__node = node

        from DSR_ROBOT2 import (
            set_tool,
            set_tcp,
            movel,
            movej,
            get_current_posx,
            DR_BASE,
            
        )
        from DR_common2 import posx, posj

        tester = VirtualBottleTipTest(
            node=node,
            set_tool=set_tool,
            set_tcp=set_tcp,
            movel=movel,
            movej=movej,
            get_current_posx=get_current_posx,
            posx=posx,
            posj=posj,
            dr_base=DR_BASE,
            
        )

        tester.run()

    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().info("Keyboard Interrupt")

    except Exception as error:
        if node is not None:
            node.get_logger().error(
                f"가상 약통 끝 테스트 실패: {error}"
            )
        else:
            print(f"가상 약통 끝 테스트 실패: {error}")
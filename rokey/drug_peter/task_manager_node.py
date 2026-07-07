# task_manager_node.py

# - `task_manager_node` (주문 및 흐름 제어 판단):
# 처방전 DB에 접근하여 처방 순서를 기록하고, 리필이 필요한 약품을 확인하여 로봇에게 작업 명령을 내리는 **메인 컨트롤러** 역할을 합니다.
#     - 자세한 설명
#         - 처방전 DB에서 처방 순서 및 약의 종류, 개수 가져오기
#         - 약의 위치(적재소),(조제실) 가져오기
#         - (적재소)약의 개수 판단해서 동작 시행 여부 결정 및 Pub (bool)
#         - 약의 개수 충족 →처방 순서, 약의 종류, 약의 위치 Pub (String 으로)
#         - DB로 사용된 약 개수 보냄 (미리 보냄 )
#         - 남은 약 개수 pub



import rclpy
from rclpy.node import Node
from time import sleep
from std_msgs.msg import Bool    
from std_msgs.msg import String
from std_msgs.msg import Float32
import json
import math 
import os
import random
from pathlib import Path

# QoS(Quality of Service) 설정을 위한 라이브러리
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy

class Task_Manager(Node):
    def __init__(self):
        super().__init__('task_manager')
       
#       ----------------------------------sub--------------------------------------------------------------
        # 남은 약 개수
        self.create_subscription(String, 'medicine_num', self._medicine_num_callback, 10) 
        # 처방전
        self.create_subscription(String, 'prescription', self._prescription_callback, 10)
        # 약 위치 좌표
        self.create_subscription(String, 'medicine_pos', self._medicine_pos_callback, 10)



#       ----------------------------------pub--------------------------------------------------------------
        # 처방전
        self.prescription = self.create_publisher(String, '/dsr01/prescription', 10)
        # 사용된 약 개수
        self.used_medicine = self.create_publisher(String, '/dsr01/used_medicine_num', 10)
        # 조제실 약 개수
        self.dispensary_a_num = self.create_publisher(Bool, '/dsr01/dispensary_a_num', 10) # A -> 타이레놀 
        self.dispensary_b_num = self.create_publisher(Bool, '/dsr01/dispensary_b_num', 10) # B -> 탁센
        self.dispensary_c_num = self.create_publisher(Bool, '/dsr01/dispensary_c_num', 10) # C -> 브루펜
        self.dispensary_d_num = self.create_publisher(Bool, '/dsr01/dispensary_d_num', 10) # D -> 활명수






    def _medicine_num_callback(self, msg: String):
        
        pass

    def _prescription_callback(self, msg: String):
        prescription = self._json_to_prescription(msg)
        if prescription is None:
            return
        pass

    def _medicine_pos_callback(self, msg: String):
        pass









def main(args=None):
    rclpy.init(args=args)
    node = Task_Manager()
    try:
        rclpy.spin(node)
        node.get_logger().info("task_manager_node 실행")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()











#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, String
import base64  # Python 표준 라이브러리 (충돌 X)
from rclpy.qos import QoSPresetProfiles

# OpenCV, NumPy 임포트
import cv2
import numpy as np

qos_profile = QoSPresetProfiles.SENSOR_DATA.value

class ConvertImageNode(Node):
    def __init__(self):
        super().__init__('convert_image_node')

        # 플래그 기본값
        self.convert_flag = False

        # /convert_flag (std_msgs/Bool) 구독
        self.flag_sub = self.create_subscription(
            Bool,
            '/convert_flag',
            self.flag_callback,
            10
        )

        # /theta/image_raw/compressed (sensor_msgs/CompressedImage) 구독
        self.image_sub = self.create_subscription(
            CompressedImage,
            #'/image_raw',
            '/theta/image_raw/compressed',
            self.image_callback,
            qos_profile
        )

        # /convert_image (std_msgs/String) 퍼블리셔
        self.convert_pub = self.create_publisher(
            String,
            '/convert_image',
            10
        )

        self.get_logger().info('ConvertImageNode started.')

    def flag_callback(self, msg: Bool):
        """/convert_flag 콜백"""
        self.convert_flag = msg.data
        self.get_logger().info(f'[flag_callback] convert_flag = {self.convert_flag}')

        # 추가: flag가 True일 때 콘솔에 메시지 출력
        if self.convert_flag:
            print("Flag is True! Start Base64 conversion.")

    def image_callback(self, msg: CompressedImage):
        """/theta/image_raw/compressed 콜백"""

        # 1) CompressedImage -> OpenCV 이미지 디코딩
        np_arr = np.frombuffer(msg.data, np.uint8)         # bytes -> numpy 배열
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # JPG/PNG 등으로 디코딩

        # 2) 디코딩된 이미지를 화면에 표시 (imshow)
        if cv_image is not None:
            cv2.imshow("Received Image", cv_image)
            cv2.waitKey(1)  # 1ms 동안 키 입력 대기 (창 업데이트용)

        # 3) convert_flag가 False이면 Base64 변환을 하지 않음
        if not self.convert_flag:
            return

        # 4) Base64 인코딩 로직
        base64_str = base64.b64encode(msg.data).decode('utf-8')
        out_msg = String()
        out_msg.data = base64_str
        self.convert_pub.publish(out_msg)

        self.get_logger().info('[image_callback] Published base64 encoded image')

def main(args=None):
    rclpy.init(args=args)
    node = ConvertImageNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        # ros2 노드 종료 시 OpenCV 창 닫기
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

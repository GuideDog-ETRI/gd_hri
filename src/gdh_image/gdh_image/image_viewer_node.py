#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

class ImageViewerNode(Node):
    def __init__(self):
        super().__init__('image_viewer')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/theta/image_raw/compressed',
            self.image_callback,
            10
        )
        self.subscription  # 변수 미사용 경고 방지

    def image_callback(self, msg):
        # 수신한 데이터(byte 배열)를 numpy 배열로 변환 후 디코딩
        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if image is not None:
            cv2.imshow("Image Viewer", image)
            cv2.waitKey(1)
        else:
            self.get_logger().error("이미지 디코딩 실패")

def main(args=None):
    rclpy.init(args=args)
    node = ImageViewerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


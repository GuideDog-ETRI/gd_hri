#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

# yaml
import yaml

class ImagePublisherNode(Node):
    def __init__(self):
        super().__init__('image_publisher')
        
        # load from conf.yaml
        path_to_config = 'models/gdh_config.yaml'
        if os.path.exists(path_to_config):
            with open(path_to_config) as fid:
                conf = yaml.full_load(fid)
        else:
            raise AssertionError(f'No gdh_config file in {path_to_config}.')
            
        # 파라미터 설정
        self.declare_parameter('image_folder', conf['image_publisher']['image_folder'])
        self.declare_parameter('publish_rate', conf['image_publisher']['publish_rate'])  # 초당 1회 publish

        self.image_folder = self.get_parameter('image_folder').value
        self.publish_rate = self.get_parameter('publish_rate').value

        self.publisher_ = self.create_publisher(CompressedImage, '/theta/image_raw/compressed', 10)

        # jpg, jpeg, png 파일만 리스트업 (정렬하여 순차적으로 전송)
        self.images = sorted([
            f for f in os.listdir(self.image_folder)
            if os.path.isfile(os.path.join(self.image_folder, f)) and f.lower().endswith(('.jpg', '.jpeg', '.png'))
        ])
        self.current_index = 0

        # OpenCV 창 생성 (키 입력 처리를 위함)
        cv2.namedWindow("Publisher Control", cv2.WINDOW_NORMAL)

        # 타이머를 사용해 1초마다 이미지를 publish함
        self.create_timer(1.0 / self.publish_rate, self.publish_callback)

    def publish_callback(self):
        if not self.images:
            self.get_logger().warn(f"'{self.image_folder}' 폴더에서 이미지 파일을 찾을 수 없습니다.")
            return

        image_path = os.path.join(self.image_folder, self.images[self.current_index])
        self.get_logger().info(f"Publishing image: {image_path}")

        # OpenCV를 사용하여 이미지를 읽기
        image = cv2.imread(image_path, cv2.IMREAD_COLOR)
        if image is None:
            self.get_logger().error(f"이미지 로드 실패: {image_path}")
            return

        # 이미지 리사이즈: height가 720이 되도록, aspect ratio 유지
        original_height, original_width = image.shape[:2]
        new_height = 720
        scale = new_height / original_height
        new_width = int(original_width * scale)
        resized_image = cv2.resize(image, (new_width, new_height))

        # OpenCV 창에 현재 이미지 표시
        cv2.imshow("Publisher Control", resized_image)

        # 파일 확장자에 따라 포맷 지정 (기본은 jpg)
        ext = os.path.splitext(image_path)[1].lower()
        img_format = '.png' if ext == '.png' else '.jpg'
        success, buffer = cv2.imencode(img_format, resized_image)
        if not success:
            self.get_logger().error("이미지 인코딩 실패")
            return

        image_data = buffer.tobytes()

        # CompressedImage 메시지 생성 및 publish
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = "png" if img_format == ".png" else "jpeg"
        msg.data = image_data

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisherNode()

    try:
        # 별도의 루프를 돌면서 키 입력을 지속적으로 폴링
        while rclpy.ok():
            # 노드의 콜백(타이머 콜백 등)들을 처리
            rclpy.spin_once(node, timeout_sec=0.05)
            # 50ms 동안 키 이벤트 확인
            key = cv2.waitKey(50)
            # OS에 따라 arrow key의 코드가 다를 수 있음
            # Linux: 왼쪽 65361, 오른쪽 65363 / Windows: 왼쪽 81, 오른쪽 83 등
            # 사용 중인 OS에 맞게 아래 조건을 조정하세요.
            if key != -1:
                if key in [65361, 81]:  # 왼쪽 화살표
                    node.current_index = (node.current_index - 1) % len(node.images)
                    node.get_logger().info("왼쪽 화살표 입력: 이전 이미지로 전환")
                elif key in [65363, 83]:  # 오른쪽 화살표
                    node.current_index = (node.current_index + 1) % len(node.images)
                    node.get_logger().info("오른쪽 화살표 입력: 다음 이미지로 전환")
    except KeyboardInterrupt:
        pass

    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/python3
import rclpy
from rclpy.node import Node

import cv2
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import threading    # for keyboard input
import numpy as np
import sys
import termios
import tty
import select


class PhotoPublisher(Node):
    def __init__(self, image_folder):
        super().__init__('photo_publisher')   # Node name

        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, '/image_raw', 10)
        
        self.image_folder = image_folder 
        self.image_files = sorted([
            os.path.join(self.image_folder, f)
            for f in os.listdir(self.image_folder)
            if f.lower().endswith(('.png', '.jpg', '.jpeg', '.bmp'))
        ])
        if not self.image_files:
            self.get_logger().error('이미지 파일을 찾을 수 없습니다.')
            rclpy.shutdown()
            return
        
        self.num_images = len(self.image_files)
        self.current_index = 0
        self.num_sel_imgs = 3
        self.get_logger().info(f'{image_folder} 폴더에서 총 {self.num_images}장의 이미지를 찾았습니다.')
        self.get_logger().info(f'이미지는 {self.num_sel_imgs}장씩 묶어서 처리합니다.')

        # 이미지 리스트를 순환 리스트로 설정
        self.images = [cv2.imread(img_file) for img_file in self.image_files]
        if any(img is None for img in self.images):
            self.get_logger().error('이미지 로딩에 실패했습니다. 노드를 종료합니다.')
            rclpy.shutdown()
            return

        # 이미지 크기 통일
        self.resize_images()

        self.timer = self.create_timer(0.5, self.timer_callback)  # Publish every second

        # 키보드 입력 스레드 시작
        self.keyboard_thread = threading.Thread(target=self.keyboard_input)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()

        self.get_logger().info('이미지 퍼블리셔 노드가 시작되었습니다.')

    def resize_images(self):
        # 첫 번째 이미지의 크기로 모든 이미지 크기를 변경
        first_image_shape = self.images[0].shape[:2]
        self.images = [cv2.resize(img, (first_image_shape[1], first_image_shape[0])) for img in self.images]

    def timer_callback(self):
        # 현재 인덱스부터 3개의 이미지 선택
        indices = [(self.current_index + i) % self.num_images for i in range(self.num_sel_imgs)]
        selected_images = [self.images[i] for i in indices]

        # 이미지를 옆으로 붙이기
        combined_image = np.hstack(selected_images)

        # 이미지 퍼블리시
        image_message = self.bridge.cv2_to_imgmsg(combined_image, encoding="bgr8")
        self.publisher.publish(image_message)
        self.get_logger().info(f'{indices[0]+1}, {indices[1]+1}, {indices[2]+1}번째 이미지를 퍼블리시했습니다.')

    # def timer_callback(self):
    #     if self.cv_image is not None:
    #         image_message = self.bridge.cv2_to_imgmsg(self.cv_image, encoding='bgr8')
    #         self.publisher.publish(image_message)
    #         self.get_logger().info('Image published')
    #     else:
    #         self.get_logger().warn('No image to publish')
        
    # def read_image(self):
    #     if os.path.exists(self.image_path):
    #         self.cv_image = cv2.imread(self.image_path)
    #         self.get_logger().info(f'Image loaded from {self.image_path}')
    #     else:
    #         self.get_logger().error(f'Image not found at {self.image_path}')
    #         self.cv_image = None

    def keyboard_input(self):
        # 터미널 설정 저장
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)

        try:
            tty.setcbreak(fd)
            while True:
                if not rclpy.ok():
                    break
                if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                    ch = sys.stdin.read(1)
                    if ch == 'n':
                        # 다음 3장으로 이동
                        self.current_index = (self.current_index + 3) % self.num_images
                        self.get_logger().info('다음 3장으로 이동합니다.')
                    elif ch == 'p':
                        # 이전 3장으로 이동
                        self.current_index = (self.current_index - 3) % self.num_images
                        self.get_logger().info('이전 3장으로 이동합니다.')
                    elif ch == 'e':
                        # 'exit' 입력 처리
                        remaining_chars = sys.stdin.read(3)
                        if remaining_chars == 'xit':
                            self.get_logger().info('노드를 종료합니다.')
                            rclpy.shutdown()
                            break
        finally:
            # 터미널 설정 복원
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

def main(args=None):
    rclpy.init(args=args)

    image_folder = '../gdh_sample_images'

    photo_publish_node = PhotoPublisher(image_folder)
    try:
        rclpy.spin(photo_publish_node)
    except KeyboardInterrupt:
        pass
    finally:
        photo_publish_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

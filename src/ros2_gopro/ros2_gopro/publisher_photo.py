#!/usr/bin/python3
import rclpy
from rclpy.node import Node

import cv2
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class PhotoPublisher(Node):
    def __init__(self, image_path):
        super().__init__('photo_publisher')   # Node name
        self.image_path = image_path
        self.read_image()

        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, '/photo', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)  # Publish every second
        
    def read_image(self):
        if os.path.exists(self.image_path):
            self.cv_image = cv2.imread(self.image_path)
            self.get_logger().info(f'Image loaded from {self.image_path}')
        else:
            self.get_logger().error(f'Image not found at {self.image_path}')
            self.cv_image = None

    def timer_callback(self):
        if self.cv_image is not None:
            image_message = self.bridge.cv2_to_imgmsg(self.cv_image, encoding='bgr8')
            self.publisher.publish(image_message)
            self.get_logger().info('Image published')
        else:
            self.get_logger().warn('No image to publish')

def main(args=None):
    rclpy.init(args=args)
    image_path = './street0.jpg'
    photo_publisher = PhotoPublisher(image_path)
    rclpy.spin(photo_publisher)
    photo_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

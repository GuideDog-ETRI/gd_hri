#!/usr/bin/python3
import rclpy
from rclpy.node import Node

import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class VideoPublisher(Node):

    def __init__(self):
        super().__init__('video_publisher')   # Node name
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)
        self.publisher = self.create_publisher(Image, '/webcam', 10)

    def run(self):
        while(self.cap.isOpened()):
            ret, frame = self.cap.read()

            if ret:
                self.publisher.publish(self.bridge.cv2_to_imgmsg(frame, 'bgr8'))
                self.get_logger().info(f'Publishing: {frame.shape}')
            else:
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            
        self.cap.release()
        

def main(args=None):
    rclpy.init(args=args)

    video_publisher = VideoPublisher()

    # rclpy.spin(minimal_publisher)   # make spin-off
    video_publisher.run()


    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    video_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

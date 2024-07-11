import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class VideoSubscriber(Node):
    def __init__(self):
        super().__init__('video_subscriber')
        qos = QoSProfile(depth=10)
        self.subscription = self.create_subscription(
            Image,
            '/webcam',
            self.listener_callback,
            qos)
        self.bridge = CvBridge()
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        cv2.imshow('frame', frame)
        cv2.waitKey(10)


def main(args=None):
    rclpy.init(args=args)

    video_subscriber = VideoSubscriber()

    rclpy.spin(video_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    video_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rclpy.qos import QoSProfile, DurabilityPolicy


class GoProSubscriber(Node):
    def __init__(self):
        super().__init__('gopro_listener')
        # qos = QoSProfile(qos)
        # qos_profile=ReliabilityPolicy.BEST_EFFORT
        qos_profile = QoSProfile(depth=10)
        
        self.subscription = self.create_subscription(
            Image,
            '/gopro',
            self.listener_callback,
            qos_profile=qos_profile)
        self.bridge = CvBridge()
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # self.get_logger().info('publisher_gopro.listener_callback')
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        print('listener_gopro: ', frame.shape)

        cv2.imshow('listener', frame)
        cv2.waitKey(10)


def main(args=None):
    rclpy.init(args=args)

    video_subscriber = GoProSubscriber()

    rclpy.spin(video_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    video_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# import cv2

# class GoProImageSubscriber(Node):
#     def __init__(self):
#         super().__init__('gopro_image_subscriber')
#         self.subscription = self.create_subscription(
#             Image,
#             '/gopro',
#             self.image_callback,
#             10)
#         self.subscription  # prevent unused variable warning
#         self.bridge = CvBridge()

#     def image_callback(self, msg):
#         cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
#         cv2.imshow("GoPro Image", cv_image)
#         cv2.waitKey(1)

# def main(args=None):
#     rclpy.init(args=args)
#     gopro_image_subscriber = GoProImageSubscriber()
#     rclpy.spin(gopro_image_subscriber)
#     gopro_image_subscriber.destroy_node()
#     cv2.destroyAllWindows()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

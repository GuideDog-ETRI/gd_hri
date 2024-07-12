import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import py360convert

import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import pdb


class VideoSubscriber(Node):
    def __init__(self):
        super().__init__('video_subscriber')
        qos = QoSProfile(depth=10)

        self.topic_name = str(sys.argv[1])

        self.subscription = self.create_subscription(
            Image,
            self.topic_name,
            self.listener_callback,
            qos)
        self.bridge = CvBridge()
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        frame_ = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        print('original: ', frame_.shape)

        if self.topic_name == '/image_raw':
            frame = self.convert_to_planar(frame_)
            print('planar: ', frame.shape)
        else:
            frame = frame_

        cv2.imshow('frame', frame)
        cv2.waitKey(10)

    def convert_to_planar(cv_image, fov_deg=(90, 70), u_deg=0, v_deg=0, out_hw=(480,640),  mode="bilinear"):
        '''
        fov_deg=(90, 70)
        u_deg=0  # hori axis
        v_deg=0  # vert axis
        out_hw=(480,640)
        mode="bilinear"
        '''
        ## Read Image
        eqImg = cv_image
        in_h, in_w, _ = eqImg.shape
        (out_h, out_w) = out_hw

        planarImg = py360convert.e2p(eqImg, fov_deg=fov_deg, u_deg=u_deg, v_deg=v_deg, out_hw=out_hw, mode=mode)
        
        return planarImg


def main(args=None):
    rclpy.init(args=args)

    print('ros2 topic list')

    video_subscriber = VideoSubscriber()

    rclpy.spin(video_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    video_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



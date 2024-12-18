import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import cv2
import py360convert
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import pdb


class VideoSubscriber(Node):
    def __init__(self):
        super().__init__('video_subscriber')
        qos = QoSProfile(depth=10)

        self.topic_name = str(sys.argv[1])
        self.hfov = int(sys.argv[2])
        self.u_deg = int(sys.argv[3])

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
            frame = self.convert_to_planar(frame_, fov_deg=(self.hfov, 70), u_deg=self.u_deg)
            print('planar: ', frame.shape)
        else:
            frame = frame_

        cv2.imshow('frame', frame)
        cv2.waitKey(10)

    def convert_to_planar(self, np_image, fov_deg=(90, 70), u_deg=0, v_deg=0, out_hw=(480,640),  mode="bilinear"):
        '''
        fov_deg=(90, 70)
        u_deg=0  # hori axis
        v_deg=0  # vert axis
        out_hw=(480,640)
        mode="bilinear"
        '''
        ## Read Image
        in_h, in_w, _ = np_image.shape
        (out_h, out_w) = out_hw

        # rescaling
        ratio_h = 640. / 90.
        new_w = int(ratio_h * fov_deg[0])
        out_hw = (out_hw[0], new_w)

        planarImg = py360convert.e2p(np_image, fov_deg=fov_deg, u_deg=u_deg, v_deg=v_deg, out_hw=out_hw, mode=mode)
        
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



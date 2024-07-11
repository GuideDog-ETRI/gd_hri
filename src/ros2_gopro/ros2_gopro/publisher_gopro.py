#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

# opencv
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# gopro
from ros2_gopro.my_webcam import MyGoProWebcamPlayer

# bluetooth connection by terminal
# hcitool dev # 1. identify the bth device on my PC
# hcitool -i hci0 scan  # 2. scan the bth device (GoPro)
# bluetoothctl
#     truch MAC add. # 3. trust the device for future connection
#     connect Mac Add   # 4. connect
# paired-devices    # 5. 

# https://gopro.github.io/OpenGoPro/http_2_0#commands-quick-reference
# res: 12=1080p(1920x1080), 7=720(1280x720), 4=480
# fov: Wide (id: 0), Narrow (id: 2), Superview (id: 3), Linear (id: 4)

class GoProPublisher(Node):

    def __init__(self):
        super().__init__('gopro_publisher')   # Node name
        
        # ros
        # print('def ros')
        # self.bridge = CvBridge()
        # self.publisher = self.create_publisher(Image, '/gopro', 10)
        self.topic_name = '/gopro'
        # self.qos_profile = 10
        self.qos_profile = QoSProfile(depth=10)
                
        # gopro
        print('def gopro')
        serial = '816'
        port = '9000'
        resolution = 12
        fov = 4
        # self.webcam1 = MyGoProWebcamPlayer(serial, port, self.bridge, self.publisher)
        self.webcam1 = MyGoProWebcamPlayer(serial, port, self.topic_name, self.qos_profile)
        
        print('def webcam enable')
        # self.webcam1.open()
        self.webcam1.webcam.enable()

        print('def webcam start')
        # self.webcam1.play(resolution, fov)
        self.webcam1.webcam.start(self.webcam1.port, resolution, fov)
        

    def run(self):
        print('run webcam1.player.start: ', MyGoProWebcamPlayer.STREAM_URL.format(port=self.webcam1.port))

        self.webcam1.player.start(
            MyGoProWebcamPlayer.STREAM_URL.format(port=self.webcam1.port)
            )
        input('press enter to stop\n\n')
        

        # gopro
        self.webcam1.close()
        print('exist run')


        # self.cap = cv2.VideoCapture(0)

        # # ros
        # while(self.cap.isOpened()):
        #     ret, frame = self.cap.read()

        #     if ret:
        #         self.publisher.publish(self.bridge.cv2_to_imgmsg(frame, 'bgr8'))
        #         self.get_logger().info(f'Publishing: {frame.shape}')
        #     else:
        #         self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            
        # self.cap.release()
        

def main(args=None):
    rclpy.init(args=args)

    print('create video_publisher')
    video_publisher = GoProPublisher()

    print('run video_publisher')
    # rclpy.spin(minimal_publisher)   # make spin-off
    video_publisher.run()


    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    print('del video_publisher')
    video_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

from gdh_interfaces.srv import AddThreeInts
from gdh_interfaces.srv import GDHInitializeDetectStaticObject, GDHTerminateDetectStaticObject
from gdh_interfaces.srv import GDHDetectStaticObjectAll, GDHDetectStaticTargetObject, GDHDetectStaticObject
from gdh_interfaces.srv import GDHExplainPathGP, GDHSpeakCodeID
from gdh_interfaces.msg import GDHDetection2DExt

import rclpy
from rclpy.node import Node

from PIL import Image
import numpy as np
from ultralytics import YOLO
from vision_msgs.msg import BoundingBox2D, Pose2D, Point2D
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rclpy.qos import QoSProfile, DurabilityPolicy

class GDHService(Node):

    def __init__(self):
        super().__init__('gdh_service')     # node_name

        qos_profile = QoSProfile(depth=10)
        # topic_name = '/gopro'
        topic_name = '/webcam'
        self.subscription = self.create_subscription(
            Image,
            topic_name,
            self.listener_callback_gopro,
            qos_profile=qos_profile)
        self.bridge = CvBridge()
        self.subscription  # prevent unused variable warning
        self.latest_gopro_image = None

        # service type(in/out params), name, callback func.
        self.srv_init_detector = self.create_service(GDHInitializeDetectStaticObject, 'GDH_init_detector', self.init_detector)
        self.srv_term_detector = self.create_service(GDHTerminateDetectStaticObject, 'GDH_term_detector', self.term_detector)
        self.srv_detect_all = self.create_service(GDHDetectStaticObjectAll, 'GDH_detect_all', self.detect_all)
        self.srv_detect = self.create_service(GDHDetectStaticObject, 'GDH_detect', self.detect)
        self.srv_detect_target = self.create_service(GDHDetectStaticTargetObject, 'GDH_detect_target', self.detect_target)
        self.srv_explain_pathgp = self.create_service(GDHExplainPathGP, 'GDH_explain_path_to_gp', self.explain_path_to_gp)
        self.srv_speak_codeid = self.create_service(GDHSpeakCodeID, 'GDH_speak_codeid', self.speak_codeid)

        self.theta_all = [90, 270]
        self.hfov_all = [45, 45]

        self.yolo_model = None
        self.yolo_conf_threshold = 0.5


    def listener_callback_gopro(self, msg):
        self.latest_gopro_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')    # nparray is returned

        self.get_logger().info(f'listener_callback: {self.latest_gopro_image.shape}')

        # cv2.imshow('listener', self.latest_gopro_image)
        # cv2.waitKey(10)


    def get_image(self, cam_type=0, theta=0, hfov=0):
        ret = True
        res_imgs = []

        print('func get_image is not implemented!')

        if cam_type == 0:
            # TODO: use omniverse camera
            if not isinstance(theta, list):
                theta = [theta]
        
            if not isinstance(hfov, list):
                hfov = [hfov]

            for th, hf in zip(theta, hfov):
                if th < 90:
                    img = Image.open("street1.jpg")
                else:
                    img = Image.open("street2.jpg")
                
                res_imgs.append(img)
        else:
            # use gopro
            # img = Image.open("street1.jpg")
            if self.latest_gopro_image is not None:
                res_imgs.append(self.latest_gopro_image)
            else:
                ret = False

        return ret, res_imgs


    def init_detector(self, request, response):
        try:
            # Load a model
            # import pdb
            # pdb.set_trace()
            repo = "models/gd_demo/train958/weights/best.pt"
            self.yolo_model = YOLO(repo)
            response.errcode = response.ERR_NONE
        except:
            self.yolo_model = None
            response.errcode = response.ERR_UNKNOWN
        
        self.get_logger().info('Incoming request @ init_detector\n\tresponse: %d,  %d(UNKNOWN), %d(NONE)' % (response.errcode, response.ERR_UNKNOWN, response.ERR_NONE))

        return response
    

    def term_detector(self, request, response):
        if self.yolo_model is not None:
            del self.yolo_model

        response.errcode = response.ERR_NONE
        
        self.get_logger().info('Incoming request @ term_detector\n\tresponse: %d,  %d(UNKNOWN), %d(NONE)' % (response.errcode, response.ERR_UNKNOWN, response.ERR_NONE))

        return response
    

    def detect_common(self, list_imgs, response):
        # run yolo            
        results = self.yolo_model.predict(source=list_imgs)
        response.detections = []

        # Process results list
        for cam_id, result in enumerate(results):
            # data = result.boxes.data
            boxes_cls = result.boxes.cls          # n_det
            boxes_conf = result.boxes.conf        # n_det
            boxes_xywh = result.boxes.xywh    # n_det x 4

            for i_box in range(len(boxes_cls)):
                conf = float(boxes_conf[i_box].item())
                box_xywh = [float(item.item()) for item in boxes_xywh[i_box]]
                cls = int(boxes_cls[i_box].item())

                if conf >= self.yolo_conf_threshold:
                    box2d = BoundingBox2D(center=Pose2D(position=Point2D(x=box_xywh[0], y=box_xywh[1]), 
                                                        theta=float(0.0)), 
                                            size_x=box_xywh[2], size_y=box_xywh[3])
                    det_res = GDHDetection2DExt(cam_id=cam_id, theta=float(0.0), hfov=float(0.0), 
                                                obj_type=cls, obj_status=0,
                                                bbox=box2d)
                    # std_msgs/Header header
                    response.detections.append(det_res)

            # result.show()  # display to screen
            result.save(filename=f'result_{cam_id}.jpg')  # save to disk

        if len(response.detections) > 0:
            response.errcode = response.ERR_NONE_AND_FIND_SUCCESS
        else:
            response.errcode = response.ERR_NONE_AND_FIND_FAILURE

        return response


    def detect_target(self, request, response):
        if self.yolo_model is None:
            response.errcode = response.ERR_NO_MODEL
        else:
            ret_img, list_imgs = self.get_image(1, theta=self.theta_all, hfov=self.hfov_all)

            if not ret_img:
                response.errcode = response.ERR_NO_IMAGE
            else:
                response = self.detect_common(list_imgs, response)

                list_target_objects = request.object_name
                detections_filtered = [item for item in response.detections 
                                    if self.yolo_model.names[int(item.obj_type)] in list_target_objects]
                response.detections = detections_filtered

                if len(response.detections) > 0:
                    response.errcode = response.ERR_NONE_AND_FIND_SUCCESS
                else:
                    response.errcode = response.ERR_NONE_AND_FIND_FAILURE

        self.get_logger().info('Incoming request @ detect_target\n\tresponse: %d,  %d(UNKNOWN), %d(NONE_AND_FAIL), %d(NONE_AND_FIND)' % 
                               (response.errcode, response.ERR_UNKNOWN, response.ERR_NONE_AND_FIND_FAILURE, response.ERR_NONE_AND_FIND_SUCCESS))

        return response
    

    def detect(self, request, response):
        if self.yolo_model is None:
            response.errcode = response.NO_MODEL
        else:
            theta = request.theta
            hfov = request.hfov

            ret_img, list_imgs = self.get_image(1, theta=[theta], hfov=[hfov])

            if not ret_img:
                response.errcode = response.NO_IMAGE
            else:
                response = self.detect_common(list_imgs, response)
              
        self.get_logger().info('Incoming request @ detect\n\tresponse: %d,  %d(UNKNOWN), %d(NONE_AND_FAIL), %d(NONE_AND_FIND)' % 
                               (response.errcode, response.ERR_UNKNOWN, response.ERR_NONE_AND_FIND_FAILURE, response.ERR_NONE_AND_FIND_SUCCESS))

        return response
    

    def detect_all(self, request, response):
        if self.yolo_model is None:
            response.errcode = response.NO_MODEL
        else:
            ret_img, list_imgs = self.get_image(1, theta=self.theta_all, hfov=self.hfov_all)

            if not ret_img:
                response.errcode = response.NO_IMAGE
            else:
                response = self.detect_common(list_imgs, response)
              
        self.get_logger().info('Incoming request @ detect_all\n\tresponse: %d,  %d(UNKNOWN), %d(NONE_AND_FAIL), %d(NONE_AND_FIND)' % 
                               (response.errcode, response.ERR_UNKNOWN, response.ERR_NONE_AND_FIND_FAILURE, response.ERR_NONE_AND_FIND_SUCCESS))

        return response
    

    def explain_path_to_gp(self, request, response):
        self.get_logger().info('TODO: not implemented\n')
        self.get_logger().info('Incoming request @ explain_path_to_gp\n\tresponse: %d' % (response.errcode))

        response.errcode = response.ERR_UNKNOWN

        return response


    def speak_codeid(self, request, response):
        self.get_logger().info('TODO: not implemented\n')
        self.get_logger().info('Incoming request @ speak_codeid\n\tresponse: %d' % (response.errcode))

        response.errcode = response.ERR_UNKNOWN
        
        return response


def main(args=None):
    rclpy.init(args=args)

    gdh_service_node = GDHService()
    rclpy.spin(gdh_service_node)
    gdh_service_node.destroy_node()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
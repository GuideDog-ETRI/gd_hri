from gd_ifc_pkg.srv import GDHInitializeDetectStaticObject, GDHTerminateDetectStaticObject
from gd_ifc_pkg.srv import GDHStartDetectObject, GDHStopDetectObject
from gd_ifc_pkg.srv import GDHExplainPathGP
from gd_ifc_pkg.msg import GDHDetection2DExt, GDHDetections

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

import os
from PIL import Image as PilImage
import numpy as np

# Ricoh Theta Z
import py360convert

# Yolo and detector
from ultralytics import YOLO
from vision_msgs.msg import BoundingBox2D, Pose2D, Point2D
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2
import threading
import math

# Heartbeat
from gd_ifc_pkg.msg import GDHStatus
from std_msgs.msg import Header
from rclpy.clock import Clock

class GDHService(Node):
    def __init__(self):
        super().__init__('gdh_service')     # node_name

        # subscription of ricoh image
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT     
        # Image        
        self.subscription = self.create_subscription(
            CompressedImage, '/theta/image_raw/compressed',
            self.listener_callback_ricoh,
            qos_profile=qos_profile)
        self.bridge = CvBridge()
        self.subscription  # prevent unused variable warning
        self.latest_ricoh_erp_image = None

        # service type(in/out params), name, callback func.
        self.srv_init_detector = self.create_service(GDHInitializeDetectStaticObject, '/GDH_init_detect', self.init_detector)
        self.srv_term_detector = self.create_service(GDHTerminateDetectStaticObject, '/GDH_term_detect', self.term_detector)

        # self.srv_detect_all = self.create_service(GDHDetectStaticObjectAll, '/GDH_detect_all', self.detect_all)
        # self.srv_detect = self.create_service(GDHDetectStaticObject, '/GDH_detect', self.detect)

        self.srv_explain_pathgp = self.create_service(GDHExplainPathGP, '/GDH_explain_path_to_gp', self.explain_path_to_gp)
        # self.srv_speak_codeid = self.create_service(GDHSpeakCodeID, '/GDH_speak_codeid', self.speak_codeid)
        
        self.srv_start_detect_target = self.create_service(GDHStartDetectObject, '/GDH_start_detect', self.start_detect_object)
        self.srv_stop_detect_target = self.create_service(GDHStopDetectObject, '/GDH_stop_detect', self.stop_detect_object)
        self.publisher_detect = self.create_publisher(GDHDetections, '/GDH_detections', 10)
        self.publisher_detect_img = self.create_publisher(Image, '/GDH_detections_img', 10)
        self.detecting = False
        self.thread = None
        
        self.htheta_list = [0, -90, 90]  # [-180, 180] in degree
        self.vtheta_list = [0, 0, 0]     
        self.hfov = 90
        self.vfov = 70
        
        self.all_object_type_id = 255
        self.detect_object_types = self.all_object_type_id

        self.yolo_model = None
        self.yolo_conf_threshold = 0.75

        self.cam_id_ricoh = 'theta_cart'

        # heartbeat status
        self.heartbeats_errcode = GDHStatus.ERR_NONE  # 오류 없음
        self.publisher_status = self.create_publisher(GDHStatus, '/GDH_status', 1)
        self.timer = self.create_timer(timer_period_sec=1.0, callback=self.timer_callback)

    def __del__(self):
        self._term_detector()

    # Heartbeat
    def timer_callback(self):
        header = Header()
        header.stamp = Clock().now().to_msg()  # 현재 시간
        header.frame_id = "gdh_status"

        # Heartbeat 메시지 생성
        status_msg = GDHStatus()
        status_msg.header = header


        status_msg.errcode = self.heartbeats_errcode
        self.heartbeats_errcode = status_msg.ERR_NONE  # 오류 없음

        self.publisher_status.publish(status_msg)
        self.get_logger().info(f"Publishing GDH status: timestamp={status_msg.header.stamp.sec}, errcode={status_msg.errcode}")

    # image
    def listener_callback_ricoh(self, msg):
    	# raw image w/o compression to numpy
        # self.latest_ricoh_erp_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')    # nparray is returned
        # compressed image to numpy
        np_arr = np.frombuffer(msg.data, np.uint8)        
        self.latest_ricoh_erp_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)    # nparray is returned
        # self.get_logger().info(f'listener_callback: msg latest_ricoh_erp_image size {self.latest_ricoh_erp_image.shape}')

    def get_rectified_ricoh_images(self, htheta_list, vtheta_list, hfov, vfov):        
        res_imgs = []

        if self.latest_ricoh_erp_image is not None:
            if not isinstance(htheta_list, list):
                htheta_list = [htheta_list]

            if not isinstance(vtheta_list, list):
                vtheta_list = [vtheta_list]
        
            ricohz_erp_image = np.copy(self.latest_ricoh_erp_image)

            for htheta, vtheta in zip(htheta_list, vtheta_list):
                planar_image = self.convert_to_planar(ricohz_erp_image, fov_deg=(hfov, vfov), 
                                                      u_deg=htheta, v_deg=vtheta)
                res_imgs.append(planar_image)
                self.get_logger().info(f'get_rectified_ricoh_images: rectified erp image size {planar_image.shape}')

            res = True
        else:
            res = False

        return res, res_imgs
    
    def convert_to_planar(self, np_image, fov_deg=(90, 70), u_deg=0, v_deg=0, out_hw=(480, 640),  mode="bilinear"):
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
    
    # Function to combine a list of NumPy images into one
    def combine_numpy_images(self, np_images):
        # Assuming all images have the same height, stack them horizontally
        combined_image = np.hstack(np_images)
        return combined_image

    # Convert the combined NumPy image into a ROS2 Image message
    def numpy_to_ros_image(self, np_image):
        # Use CvBridge to convert from OpenCV image (NumPy array) to ROS Image message
        bridge = CvBridge()
        ros_image = bridge.cv2_to_imgmsg(np_image, encoding="bgr8")
        return ros_image

    def _init_detector(self):
        # Load a model
        repo = "models/gd_demo/train958/weights/best.pt"

        if os.path.exists(repo):
            self.yolo_model = YOLO(repo)
            res = True
            self.get_logger().info(f'init_detector: yolo_model is loaded from {repo}.')
        else:
            self.yolo_model = None
            res = False
            self.get_logger().info(f'init_detector: yolo_model is not loaded. Path({repo}) is not exist.')

        return res

    def init_detector(self, request, response):
        res = self._init_detector()

        if res:
            response.errcode = response.ERR_NONE
        else:
            response.errcode = response.ERR_UNKNOWN
        
        self.get_logger().info('Incoming request @ init_detector\n\tresponse: %d,  %d(UNKNOWN), %d(NONE)' % (response.errcode, response.ERR_UNKNOWN, response.ERR_NONE))

        return response
    
    def _term_detector(self):
        if self.yolo_model is not None:
            del self.yolo_model
            self.yolo_model = None
            self.get_logger().info('term_detector: yolo_model is unloaded.')

        return True

    def term_detector(self, request, response):
        res = self._term_detector()

        response.errcode = response.ERR_NONE
        
        self.get_logger().info('Incoming request @ term_detector\n\tresponse: %d,  %d(UNKNOWN), %d(NONE)' % (response.errcode, response.ERR_UNKNOWN, response.ERR_NONE))

        return response
    
    def det_result_to_gdi_code(self, class_index):
        # class_index to gdi object index and status index
        #  
        #{0: 'door closed', 1: 'door semi-open', 2: 'door open', 
        # 3: 'pedestrian traffic light red', 4: 'pedestrian traffic light green', 
        # 5: 'stairs', 6: 'escalator', 7: 'subway entrance', 
        # 8: 'automatic door', 9: 'elevator door', 10: 'elevator button', 11: 'subway ticket gate each', 
        # 12: 'subway ticket gate handicap', 13: 'subway ticket gate all', 
        # 14: 'subway screen door'}

        # class_name = self.yolo_model.names[int(class_index)]

        res_obj_status = 0
        if class_index in [0, 1, 2]:
            res_obj_type = 10   # DOOR
            if class_index == 0:
                res_obj_status = 4  # closed
            elif class_index == 1:
                res_obj_status = 5  # semi-open
            elif class_index == 2:
                res_obj_status = 3  # open

        elif class_index in [3, 4]:
            res_obj_type = 61   # PEDESTRIAN_TRAFFIC_LIGHT
            if class_index == 3:
                res_obj_status = 10     # RED
            else:
                res_obj_status = 11     # GREEN
        elif class_index == 5:
            res_obj_type = 50   # STAIRS
        elif class_index == 6:
            res_obj_type = 40   # ESCALATOR
        elif class_index == 7:
            res_obj_type = 70   # SUBWAY_GATE
        elif class_index == 8:
            res_obj_type = 20   # AUTOMATIC_DOOR
        elif class_index == 9:
            res_obj_type = 30   # ELEVATOR_DOOR
        elif class_index == 10:
            res_obj_type = 31   # ELEVATOR_BUTTON
        elif class_index == 11:
            res_obj_type = 80   # 
        elif class_index == 12:
            res_obj_type = 81   # 
        elif class_index == 13:
            res_obj_type = 82   # 
        elif class_index == 14:
            res_obj_type = 90   # SUBWAY_SCREEN_DOOR

        return res_obj_type, res_obj_status

    def detect_common(self, list_imgs, list_img_infos):
        # dets_msg
        dets_msg = GDHDetections()
        dets_np_img = []

        header = Header()
        header.stamp = Clock().now().to_msg()  # 현재 시간
        header.frame_id = "none"
        dets_msg.header = header
        dets_msg.detections = []

        # run yolo            
        results = self.yolo_model.predict(source=list_imgs)

        # Process results list
        for idx, result in enumerate(results):
            # data = result.boxes.data
            boxes_cls = result.boxes.cls          # n_det
            boxes_conf = result.boxes.conf        # n_det
            boxes_xywh = result.boxes.xywh    # n_det x 4

            # parsing result
            cam_id = list_img_infos[idx]['cam_id']
            img_w = list_img_infos[idx]['img_w']
            img_h = list_img_infos[idx]['img_h']
            htheta_rad = math.radians(list_img_infos[idx]['htheta'])      # deg to rad
            vtheta_rad = math.radians(list_img_infos[idx]['vtheta'])      # deg to rad
            hfov_rad = math.radians(list_img_infos[idx]['hfov'])          # deg to rad
            vfov_rad = math.radians(list_img_infos[idx]['vfov'])          # deg to rad


            for i_box in range(len(boxes_cls)):
                conf = float(boxes_conf[i_box].item())
                box_xywh = [float(item.item()) for item in boxes_xywh[i_box]]
                cls = int(boxes_cls[i_box].item())

                obj_type, obj_status = self.det_result_to_gdi_code(cls)

                if conf >= self.yolo_conf_threshold:
                    box2d = BoundingBox2D(center=Pose2D(position=Point2D(x=box_xywh[0], y=box_xywh[1]), 
                                                        theta=float(0.0)), 
                                            size_x=box_xywh[2], size_y=box_xywh[3])
                    det_res = GDHDetection2DExt(header=header, bbox=box2d, 
                                                obj_type=obj_type, obj_status=obj_status,
                                                cam_id=cam_id, img_w=img_w, img_h=img_h, 
                                                htheta=htheta_rad, vtheta=vtheta_rad, 
                                                hfov=hfov_rad, vfov=vfov_rad
                                                )
                    # std_msgs/Header header
                    dets_msg.detections.append(det_res)

            # result.show()  # display to screen
            # result.save(filename=f'result_{cam_id}_{htheta}_{hfov}.jpg')  # save to disk
            dets_np_img.append(result.plot())  # save as numpy

        combined_pil_img = self.combine_numpy_images(dets_np_img)

        if len(dets_msg.detections) > 0:
            dets_msg.errcode = dets_msg.ERR_NONE_AND_FIND_SUCCESS
        else:
            dets_msg.errcode = dets_msg.ERR_NONE_AND_FIND_FAILURE

        return dets_msg, combined_pil_img
    
    def start_detect_object(self, request, response):
        if self.yolo_model is None:
            self._init_detector()
        
        if self.yolo_model is None:
            response.success = False
            response.message = 'Cannot initialize detector.'
        else:
            if not self.detecting:
                self.detect_object_types = request.object_types
                self.detecting = True
                self.thread = threading.Thread(target=self.detect_loop)
                self.thread.start()
                response.success = True
                response.message = 'Detection started.'
            else:
                response.success = False
                response.message = 'Detection is already running.'
        
        return response
    
    def stop_detect_object(self, request, response):
        if self.detecting:
            self.detecting = False
            if self.thread is not None:
                self.thread.join()
            response.success = True
            response.message = 'Detection stopped.'
        else:
            response.success = False
            response.message = 'Detection is not running.'

        return response
    
    
    def detect_loop(self):
        rate = self.create_rate(10) # 10 Hz rate
        target_object_types = self.detect_object_types

        while self.detecting:
            # get image from predefined cameras
            res, list_imgs = self.get_rectified_ricoh_images(htheta_list=self.htheta_list, vtheta_list=self.vtheta_list,
                                                                 hfov=self.hfov, vfov=self.vfov)
            if res:
                list_img_infos = []
                for idx, img in enumerate(list_imgs):
                    img_infos = {
                        'cam_id': self.cam_id_ricoh,
                        'img_w': img.shape[1],
                        'img_h': img.shape[0],
                        'htheta': self.htheta_list[idx],
                        'vtheta': self.vtheta_list[idx],
                        'hfov': self.hfov,
                        'vfov': self.vfov
                    }
                    list_img_infos.append(img_infos)

                dets_msg, combined_np_img = self.detect_common(list_imgs, list_img_infos)

                if target_object_types == self.all_object_type_id:
                    detections_filtered = dets_msg.detections
                else:
                    detections_filtered = [item for item in dets_msg.detections 
                                        if int(item.obj_type) in target_object_types]
                    
                dets_msg.detections = detections_filtered

                if len(dets_msg.detections) > 0:
                    dets_msg.errcode = dets_msg.ERR_NONE_AND_FIND_SUCCESS
                else:
                    dets_msg.errcode = dets_msg.ERR_NONE_AND_FIND_FAILURE
            
                # set heartbeats error code
                self.heartbeats_errcode = dets_msg.errcode
                
                # 결과를 Image 메시지로 변환하여 퍼블리시            
                dets_ros_img = self.numpy_to_ros_image(combined_np_img)

                self.publisher_detect.publish(dets_msg)
                self.publisher_detect_img.publish(dets_ros_img)

                self.get_logger().info('Publishing dets_msg\n\terrcode: %d,  %d(UNKNOWN), %d(FIND_FAIL), %d(FIND_SUCCESS)' % 
                        (dets_msg.errcode, dets_msg.ERR_UNKNOWN, dets_msg.ERR_NONE_AND_FIND_FAILURE, dets_msg.ERR_NONE_AND_FIND_SUCCESS))
            else:
                self.heartbeats_errcode = GDHStatus.ERR_NO_IMAGE
                self.get_logger().info('Set heartbeats_errcode as %d' % (GDHStatus.ERR_NO_IMAGE))

            rate.sleep()
        
    def explain_path_to_gp(self, request, response):
        self.get_logger().info('TODO: not implemented\n')
        self.get_logger().info('Incoming request @ explain_path_to_gp\n\tresponse: %d' % (response.errcode))

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

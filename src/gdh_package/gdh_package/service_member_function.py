from gd_ifc_pkg.srv import GDHInitializeDetectStaticObject, GDHTerminateDetectStaticObject
from gd_ifc_pkg.srv import GDHStartDetectObject, GDHStopDetectObject
from gd_ifc_pkg.srv import GDHExplainPathGP
from gd_ifc_pkg.msg import GDHDetection2DExt, GDHDetections
from gd_ifc_pkg.srv import GDGGetImageGuidancePoint

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

import os
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

import queue

# Yolo-world
from ultralytics import YOLOWorld

# Heartbeat
from gd_ifc_pkg.msg import GDHStatus
from std_msgs.msg import Header

# depth-estimation
from transformers import pipeline
from PIL import Image as PilImage

# yaml
import yaml

import time

from .erp_rectify_fast_caching import erp_to_rect

# Object type and status
from enum import IntEnum
from ctypes import c_uint8
class ObjectType(IntEnum):
    DOOR = c_uint8(10).value
    # AUTOMATIC_DOOR = c_uint8(20).value
    ELEVATOR_DOOR = c_uint8(11).value
    # ELEVATOR_BUTTON = c_uint8(31).value
    STAIRS = c_uint8(12).value
    ESCALATOR = c_uint8(13).value    
    SUBWAY_GATE = c_uint8(15).value
    SUBWAY_TICKET_GATE_EACH = c_uint8(16).value
    # SUBWAY_TICKET_GATE_HANDICAP = c_uint8(81).value
    # SUBWAY_TICKET_GATE_ALL = c_uint8(82).value
    PEDESTRIAN_TRAFFIC_LIGHT = c_uint8(17).value
    SUBWAY_SCREEN_DOOR = c_uint8(18).value
    IGNORE = c_uint8(255).value

class ObjectStatus(IntEnum):
    OPEN = c_uint8(3).value
    CLOSED = c_uint8(4).value
    SEMIOPEN = c_uint8(5).value
    RED = c_uint8(10).value
    GREEN = c_uint8(11).value
    UNKNOWN = c_uint8(99).value


class GDHService(Node):
    def __init__(self):
        super().__init__('gdh_service')     # node_name

        # load from conf.yaml
        path_to_config = 'models/gdh_config.yaml'
        if os.path.exists(path_to_config):
            with open(path_to_config) as fid:
                conf = yaml.full_load(fid)
        else:
            raise AssertionError(f'No gdh_config file in {path_to_config}.')

        self.input_type = 'ricoh_comp'   # ['rgb_raw' | 'ricoh_raw' | 'ricoh_comp' ]        
        assert self.input_type == 'ricoh_comp'
        # 'rgb_raw': for testing GDH individual functions. Image are published from folders.
        # 'ricoh_raw': for testing ROS communications of GDH
        # 'rocoh_comp': for deployment
        
        self.subscription = self.handle_subs_ricoh_comp()
        
        self.bridge = CvBridge()
        self.subscription  # prevent unused variable warning
        self.latest_ricoh_erp_image = None
        
        # service type(in/out params), name, callback func.
        self.srv_init_detector = self.create_service(GDHInitializeDetectStaticObject, '/GDH_init_detect', self.init_detector)
        self.srv_term_detector = self.create_service(GDHTerminateDetectStaticObject, '/GDH_term_detect', self.term_detector)

        self.srv_explain_pathgp = self.create_service(GDHExplainPathGP, '/GDH_explain_path_to_gp', self.explain_path_to_gp)
        
        self.srv_start_detect_target = self.create_service(GDHStartDetectObject, '/GDH_start_detect', self.start_detect_object)
        self.srv_stop_detect_target = self.create_service(GDHStopDetectObject, '/GDH_stop_detect', self.stop_detect_object)
        self.publisher_detect = self.create_publisher(GDHDetections, '/GDH_detections', 10)
        self.publisher_detect_img = self.create_publisher(Image, '/GDH_detections_img', 10)
        self.detecting = False
        self.thread = None
        self.shutdown_event = threading.Event()  # Event to signal shutdown
        
        # Forward is 180 in GDG
        self.cam_id_ricoh = conf['camera']['id']
        # self.htheta_list = [90, 180, -90]  # [-180, 180] in degree, 
        self.htheta_list = conf['camera']['htheta_list']
        self.vtheta_list = [0] * len(self.htheta_list)
        # self.hfov = 90
        self.hfov = conf['camera']['hfov']
        self.vfov = 70
        
        self.all_object_type_id = 255
        self.detect_object_types = self.all_object_type_id

        self.yolo_model = None
        self.yolo_conf_threshold = conf['yolo_model']['conf_threshold']
        self.yolo_repo = conf['yolo_model']['repo']
        self.resize_long_px = conf['yolo_model']['resize_long_px']
        self.convert_to_planar_out_h = conf['yolo_model']['convert_to_planar']['out_h']
        self.convert_to_planar_out_w = conf['yolo_model']['convert_to_planar']['out_w']
        self.use_yoloworld = conf['yolo_model']['use_yoloworld']
        self.yoloworld_repo = conf['yolo_model']['world_repo']

        self.pub_msg_subway_gate_each = conf['pub_msg']['subway_gate_each']
        self.pub_msg_subway_gate_all = conf['pub_msg']['subway_gate_all']
        self.pub_msg_subway_gate_handicap = conf['pub_msg']['subway_gate_handicap']

        self.testing_w_GDG = conf['misc']['draw_gp']

        self.debug_display_yolo = conf['debug']['display_yolo']

        # heartbeat status - yochin: temporary disabled to test scenario
        self.heartbeats_det_errcode = GDHStatus.ERR_NONE  # 오류 없음
        self.publisher_status = self.create_publisher(GDHStatus, '/GDH_status', 1)
        self.timer = self.create_timer(timer_period_sec=1.0, callback=self.timer_callback)

        # classify door status (open / close) with depth estimation
        self.check_door_status = conf['elevator_door_status']['do_check']
        self.get_logger().info(f"Check_door_status: {self.check_door_status}")

        if self.check_door_status:
            self.pipe_depth_est = pipeline(task="depth-estimation", model=conf['elevator_door_status']['depth_repo'])
            self.get_logger().info(f"Initialized self.pipe_depth_est")

        # Store depth information and bounding boxes for multiple elevators
        self.elevator_depth_buffers = {}
        self.elevator_bboxes = {}
        self.elevator_bboxes_info = {}
        self.elevator_last_seen = {}

        # Start detection loop
        if self.yolo_model is None:
            self._init_detector()   # load model from pt file

        if self.thread is None or not self.thread.is_alive():  # 스레드가 없거나 종료된 상태인지 확인
            self.detecting = False
            self.shutdown_event.clear()  # Clear shutdown event before starting
            self.thread = threading.Thread(target=self.detect_loop)
            self.thread.start()

        self.get_logger().info(f"End of the GDH.__init__")

    def destroy_node(self):
        self.get_logger().info('Node is shutting down, stopping detection thread if running.')
        self.detecting = False
        self.shutdown_event.set()
        if self.thread is not None and self.thread.is_alive():
            self.thread.join(timeout=5)
            if self.thread.is_alive():
                self.get_logger().warn('Detection thread did not exit in time during node shutdown.')
        
        self._term_detector()

        super().destroy_node()

    def handle_subs_ricoh_comp(self):
        qos_profile = QoSProfile(depth=10)
        # Compressed ERP Image        
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT     
        subscription = self.create_subscription(
            CompressedImage, '/theta/image_raw/compressed',
            self.listener_callback_ricoh_comprssed,
            qos_profile=qos_profile)
        
        return subscription
    
    
    # Heartbeat
    def timer_callback(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()  # 현재 시간
        header.frame_id = "gdh_status"

        # Heartbeat 메시지 생성
        status_msg = GDHStatus()
        status_msg.header = header

        if self.detecting:
            status_msg.errcode = self.heartbeats_det_errcode
        else:
            if self.latest_ricoh_erp_image is None:
                status_msg.errcode = status_msg.ERR_NO_IMAGE    # No Images in all image containers

                self.subscription = self.handle_subs_ricoh_comp()
                self.get_logger().info(f"Try to subscribing image in timer_callback!")
            else:
                status_msg.errcode = status_msg.ERR_NONE  # 오류 없음


        self.publisher_status.publish(status_msg)
        self.get_logger().info(f"Publishing GDH status: timestamp={status_msg.header.stamp.sec}, errcode={status_msg.errcode}")

    # image
    def listener_callback_ricoh_comprssed(self, msg):
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
            
            for htheta, vtheta in zip(htheta_list, vtheta_list):
                self.get_logger().debug(f'erp_to_rect args: self.latest_ricoh_erp_image: {self.latest_ricoh_erp_image.shape}, fov_deg_hv: ({hfov}, {vfov}), htheta: {htheta}')
                planar_image = erp_to_rect(erp_image=self.latest_ricoh_erp_image, 
                                           theta=np.deg2rad(htheta),
                                           hfov=np.deg2rad(hfov),
                                           vfov=np.deg2rad(vfov)
                                          )
                self.get_logger().debug(f'erp_to_rect outs: planar_image: {planar_image.shape}')
                                                      
                res_imgs.append(planar_image)

            res = True
        else:
            res = False

        return res, res_imgs
    
    def split_image(self, image, n):
        """
        주어진 이미지를 n개의 동일한 폭을 가진 이미지로 나눕니다.

        Args:
            image (numpy.ndarray): 원본 이미지 배열 (높이 x 너비 x 채널)
            n (int): 분할할 이미지의 개수

        Returns:
            list of numpy.ndarray: 분할된 이미지 배열의 리스트
        """
        # 이미지의 높이, 너비, 채널 수 확인
        height, width, channels = image.shape

        # 너비를 n으로 나누어 각 이미지의 폭 계산
        # 만약 나누어 떨어지지 않는다면 앞쪽 이미지에 한 픽셀씩 더 추가
        widths = [width // n + (1 if x < width % n else 0) for x in range(n)]

        # 분할할 이미지의 시작과 끝 인덱스 계산
        start_indices = [sum(widths[:i]) for i in range(n)]
        end_indices = [start + w for start, w in zip(start_indices, widths)]

        # 이미지 분할
        split_images = [image[:, start:end, :] for start, end in zip(start_indices, end_indices)]

        return split_images
    
    def convert_to_planar(self, np_image, fov_deg=(90, 70), u_deg=0, v_deg=0, out_hw=(480, 640),  mode="bilinear"):
        '''
        fov_deg=(90, 70)	# (w, h)
        u_deg=0  # hori axis
        v_deg=0  # vert axis
        out_hw=(480,640)
        mode="bilinear"
        '''
        ## Read Image
        in_h, in_w, _ = np_image.shape
        (out_h, out_w) = out_hw

        # # rescaling
        # ratio_h = 640. / 90.
        # new_w = int(ratio_h * fov_deg[0])
        # out_hw = (out_hw[0], new_w)

        self.get_logger().info(f'py360convert args: np_image: {np_image.shape}, fov_deg: {fov_deg}, out_hw: {out_hw}')
        planarImg = py360convert.e2p(np_image, fov_deg=fov_deg, u_deg=u_deg, v_deg=v_deg, out_hw=out_hw, mode=mode)
        self.get_logger().info(f'py360convert outs: planarImg: {planarImg.shape}')
        
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
        if os.path.exists(self.yolo_repo):
            self.yolo_model = YOLO(self.yolo_repo)

            if self.use_yoloworld:
                self.yoloworld_model = YOLOWorld(self.yoloworld_repo)
                self.yoloworld_model.set_classes(["control gate", "access control gate"])
                self.get_logger().info(f'init_detector: yoloworld_model is loaded from {self.yoloworld_repo}.')
            else:
                self.yoloworld_model = None
                self.get_logger().info(f'init_detector: yoloworld_model is not loaded.')

            res = True
            self.get_logger().info(f'init_detector: yolo_model is loaded from {self.yolo_repo}.')
        else:
            self.yolo_model = None
            self.yoloworld_model = None
            res = False
            self.get_logger().info(f'init_detector: yolo_model is not loaded. Path({self.yolo_repo}) is not exist.')

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

        if self.yoloworld_model is not None:
            del self.yoloworld_model
            self.yoloworld_model = None
            self.get_logger().info('term_detector: yoloworld_model is unloaded.')

        return True

    def term_detector(self, request, response):
        res = self._term_detector()

        response.errcode = response.ERR_NONE
        
        self.get_logger().info('Incoming request @ term_detector\n\tresponse: %d,  %d(UNKNOWN), %d(NONE)' % (response.errcode, response.ERR_UNKNOWN, response.ERR_NONE))

        return response
    
    def display_depth(self, np_depth):
        # 깊이 이미지 정규화 (0~255로 스케일링)
        normalized_depth_image = cv2.normalize(np_depth, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)

        # 컬러맵 적용하여 시각화 (COLORMAP_JET 사용)
        colorized_depth_image = cv2.applyColorMap(normalized_depth_image, cv2.COLORMAP_JET)

        return colorized_depth_image

    
    def is_door_open(self, np_depth, box_xywh, depth_buffer, depth_change_threshold=0.2):
        # box_xywh is in (x_center, y_center, width, height) format
        cx, cy, w, h = box_xywh

        # Convert (cx, cy, w, h) to (x_min, y_min, x_max, y_max)
        x_min = int(cx - w / 2)
        y_min = int(cy - h / 2)
        x_max = int(cx + w / 2)
        y_max = int(cy + h / 2)

        depth_roi = np_depth[y_min:y_max, x_min:x_max]
        depth_roi_resized = np.resize(depth_roi, (50, 25))

        avg_depth = np.mean(depth_roi_resized[np.isfinite(depth_roi_resized)])

        self.get_logger().info(f'is_door_open: cur_avg_depth {avg_depth}, {box_xywh}')        
        if len(depth_buffer) > 0:
            self.get_logger().info(f'is_door_open: prev_avg_depth {depth_buffer[-1]}')

            depth_diff = avg_depth - depth_buffer[-1]
            if depth_diff > depth_change_threshold:
                return True

        depth_buffer.append(avg_depth)

        # from datetime import datetime
        # current_time = datetime.now().strftime("%Y%m%d_%H%M%S")

        # np_depth_disp = self.display_depth(np_depth)
        # output_filename = f'./{current_time}_depth.jpg'
        # cv2.imwrite(output_filename, np_depth_disp)

        # # np_depth_roi_disp = self.display_depth(depth_roi)
        # output_filename = f'./{current_time}_depth_roi.jpg'
        # cv2.imwrite(output_filename, np_depth_disp[y_min:y_max, x_min:x_max])

        # # Save depth ROI as a text file for human readability
        # depth_roi_filename = f'./{current_time}_depth_roi.txt'
        # np.savetxt(depth_roi_filename, depth_roi_resized, fmt='%.2f')

        return False
    
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

        res_obj_status = ObjectStatus.UNKNOWN
        if class_index in [0, 1, 2]:    # Closed / Semi-open / Open DOOR
            res_obj_type = ObjectType.DOOR   # DOOR
            if class_index == 0:
                res_obj_status = ObjectStatus.CLOSED  # closed
            elif class_index == 1:
                res_obj_status = ObjectStatus.SEMIOPEN  # semi-open
            elif class_index == 2:
                res_obj_status = ObjectStatus.OPEN  # open

        elif class_index in [3, 4]:     # PEDESTRIAN_TRAFFIC_LIGHT
            res_obj_type = ObjectType.PEDESTRIAN_TRAFFIC_LIGHT
            if class_index == 3:
                res_obj_status = ObjectStatus.RED
            else:
                res_obj_status = ObjectStatus.GREEN
        elif class_index == 5:
            res_obj_type = ObjectType.STAIRS 
        elif class_index == 6:
            res_obj_type = ObjectType.ESCALATOR
        elif class_index == 7:
            res_obj_type = ObjectType.SUBWAY_GATE
        elif class_index == 8:  # AUTOMATIC_DOOR
            # res_obj_type = ObjectType.AUTOMATIC_DOOR
            res_obj_type = ObjectType.DOOR
            res_obj_status = ObjectStatus.CLOSED
        elif class_index == 9:
            res_obj_type = ObjectType.ELEVATOR_DOOR
            res_obj_status = ObjectStatus.CLOSED
        elif class_index == 10:
            # res_obj_type = ObjectType.ELEVATOR_BUTTON
            res_obj_type = ObjectType.IGNORE
        # elif class_index == 11:
        #     res_obj_type = ObjectType.SUBWAY_TICKET_GATE_EACH
        # elif class_index == 12:
        #     # res_obj_type = ObjectType.SUBWAY_TICKET_GATE_HANDICAP
        #     res_obj_type = ObjectType.SUBWAY_TICKET_GATE_EACH
        # elif class_index == 13:
        #     # res_obj_type = ObjectType.SUBWAY_TICKET_GATE_ALL
        #     res_obj_type = ObjectType.SUBWAY_TICKET_GATE_EACH
        elif class_index in [11, 12, 13]:
            res_obj_type = ObjectType.IGNORE

            if class_index == 11 and self.pub_msg_subway_gate_each:
                res_obj_type = ObjectType.SUBWAY_TICKET_GATE_EACH
            if class_index == 12 and self.pub_msg_subway_gate_handicap:
                # res_obj_type = ObjectType.SUBWAY_TICKET_GATE_HANDICAP
                res_obj_type = ObjectType.SUBWAY_TICKET_GATE_EACH
            if class_index == 13 and self.pub_msg_subway_gate_all:
                # res_obj_type = ObjectType.SUBWAY_TICKET_GATE_ALL
                res_obj_type = ObjectType.SUBWAY_TICKET_GATE_EACH
                
        elif class_index == 14:
            res_obj_type = ObjectType.SUBWAY_SCREEN_DOOR

        return res_obj_type, res_obj_status

    def match_existing_elevator(self, new_cam_idx, new_bbox, iou_threshold=0.3):
        def calculate_iou(bbox1, bbox2):
            # bbox1, bbox2 are in (x_center, y_center, width, height) format
            x1_center, y1_center, w1, h1 = bbox1
            x2_center, y2_center, w2, h2 = bbox2

            # Convert (x_center, y_center, width, height) to (x_min, y_min, x_max, y_max)
            x1_min = x1_center - w1 / 2
            y1_min = y1_center - h1 / 2
            x1_max = x1_center + w1 / 2
            y1_max = y1_center + h1 / 2

            x2_min = x2_center - w2 / 2
            y2_min = y2_center - h2 / 2
            x2_max = x2_center + w2 / 2
            y2_max = y2_center + h2 / 2

            # Calculate the (x, y) coordinates of the intersection rectangle
            inter_x_min = max(x1_min, x2_min)
            inter_y_min = max(y1_min, y2_min)
            inter_x_max = min(x1_max, x2_max)
            inter_y_max = min(y1_max, y2_max)

            # Check if there is no overlap
            if inter_x_max < inter_x_min or inter_y_max < inter_y_min:
                return 0.0

            # Calculate intersection area
            inter_area = (inter_x_max - inter_x_min) * (inter_y_max - inter_y_min)

            # Calculate area of each bounding box
            bbox1_area = w1 * h1
            bbox2_area = w2 * h2

            # Calculate IoU
            iou = inter_area / float(bbox1_area + bbox2_area - inter_area)

            return iou

        for elevator_id, idx_and_bbox in self.elevator_bboxes.items():
            cam_idx, bbox = idx_and_bbox

            if cam_idx == new_cam_idx:
                iou = calculate_iou(bbox, new_bbox)
                if iou > iou_threshold:
                    return elevator_id
        return None
    
    def detect_common_and_draw_gp(self, dets_msg, list_imgs, list_img_infos):
        dets_np_img = []

        # for testing with GDG
        if self.testing_w_GDG:
            self.get_point_client = None
            self.service_available = False  # 서비스 사용 가능 여부를 추적

        # run yolo
        if self.resize_long_px <= 0:
            results = self.yolo_model.predict(source=list_imgs)
        else:
            results = self.yolo_model.predict(source=list_imgs, imgsz=self.resize_long_px)
        
        if self.yoloworld_model is not None:
            results_yoloworld = self.yoloworld_model.predict(source=list_imgs,
                                                             imgsz=self.resize_long_px)

        list_np_depth_imgs = []
        if self.check_door_status:
            # numpy to pil
            for ith_depth, np_img in enumerate(list_imgs):
                pil_img = PilImage.fromarray(np_img)
                pil_depth_res = self.pipe_depth_est(pil_img)
                pil_depth = pil_depth_res['predicted_depth']        # ['predicted_depth': np.2dim, 'depth': pil_image]
                np_depth = np.array(pil_depth)
                list_np_depth_imgs.append(np_depth)

            # self.get_logger().info(f'img_shape: {list_imgs[0].shape}')
            # self.get_logger().info(f'depth_shape: {list_np_depth_imgs[0].shape}')
                
        detected_elevator_ids = set()

        # Process results list
        for idx, result in enumerate(results):
            # data = result.boxes.data
            boxes_cls = result.boxes.cls          # n_det
            boxes_conf = result.boxes.conf        # n_det
            boxes_xywh = result.boxes.xywh    # n_det x 4 (cx, cy, w, h)

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

                if obj_type != ObjectType.IGNORE:
                    if self.check_door_status and obj_type == ObjectType.ELEVATOR_DOOR:
                        matched_elevator_id = self.match_existing_elevator(idx, box_xywh)
                        if matched_elevator_id:
                            elevator_id = matched_elevator_id
                        else:
                            elevator_id = f"elevator_{len(self.elevator_bboxes) + 1}"
                            self.elevator_depth_buffers[elevator_id] = []

                        depth_buffer = self.elevator_depth_buffers[elevator_id]
                        if self.is_door_open(list_np_depth_imgs[idx], box_xywh, depth_buffer, depth_change_threshold=0.2):
                            obj_status = ObjectStatus.OPEN
                        else:
                            obj_status = ObjectStatus.CLOSED
                        self.get_logger().info(f'detected elevators: {obj_status}')

                        # Update the bounding box and last seen time
                        self.elevator_bboxes[elevator_id] = [idx, box_xywh]
                        self.elevator_bboxes_info[elevator_id] = [idx, cam_id, img_w, img_h, htheta_rad, vtheta_rad, hfov_rad, vfov_rad]

                        self.elevator_last_seen[elevator_id] = self.get_clock().now()
                        detected_elevator_ids.add(elevator_id)
                    else:
                        elevator_id = None

                    if conf >= self.yolo_conf_threshold:
                        box2d = BoundingBox2D(center=Pose2D(position=Point2D(x=box_xywh[0], y=box_xywh[1]), 
                                                            theta=float(0.0)), 
                                                size_x=box_xywh[2], size_y=box_xywh[3])
                        det_res = GDHDetection2DExt(header=dets_msg.header, bbox=box2d, 
                                                    obj_type=obj_type, obj_status=obj_status,
                                                    cam_id=cam_id, img_w=img_w, img_h=img_h, 
                                                    htheta=htheta_rad, vtheta=vtheta_rad, 
                                                    hfov=hfov_rad, vfov=vfov_rad
                                                    )
                        # std_msgs/Header header
                        dets_msg.detections.append(det_res)

            # save as numpy
            np_result = result.plot()

            if self.testing_w_GDG:
                # 서비스 클라이언트가 아직 생성되지 않았다면 생성
                if self.get_point_client is None:
                    self.get_point_client = self.create_client(GDGGetImageGuidancePoint, '/get_image_gp')
                    try:
                        self.get_logger().info('Trying to create_client with srv name, GDG - get_image_gp')
                        if not self.get_point_client.wait_for_service(timeout_sec=1.0):
                            self.get_logger().error('Get GP Service is not available!')
                            self.service_available = False  # 서비스가 사용 불가능한 경우 처리
                        else:
                            self.get_logger().info('Connected to create_client with srv name, GDG - get_image_gp')
                            self.service_available = True  # 서비스 사용 가능 상태로 설정
                    except Exception as e:
                        self.get_logger().error(f'Failed to wait for service: {str(e)}')
                        self.service_available = False

                # 서비스가 사용 가능한 경우에만 요청
                if self.service_available:
                    srv_request = GDGGetImageGuidancePoint.Request()
                    srv_request.cam_id = cam_id
                    srv_request.img_w = img_w
                    srv_request.img_h = img_h
                    srv_request.htheta = htheta_rad
                    srv_request.vtheta = vtheta_rad
                    srv_request.hfov = hfov_rad
                    srv_request.vfov = vfov_rad

                    # 서비스 호출
                    try:
                        self.get_logger().info(f'Trying to get_point_client!')
                        # future = self.get_point_client.call_async(srv_request)
                        # rclpy.spin_until_future_complete(self, future)
                        # response = future.result()
                        response = self.get_point_client.call(srv_request)

                        if response.errcode == response.ERR_NONE:
                            # Draw the point on the combined image
                            np_result = self.draw_point_on_image(np_result, response.x, response.y)
                            self.get_logger().info(f'Success to draw guidance point, {response.x}, {response.y}')
                        else:
                            self.get_logger().warn(f'Failed to get drawable guidance point: {response.errcode}')
                    except Exception as e:
                        self.get_logger().error(f'Failed to call guidance point service: {str(e)}')

            dets_np_img.append(np_result)

        combined_pil_img = self.combine_numpy_images(dets_np_img)

        # Maintain the bounding box for undetected elevators
        if self.check_door_status:
            current_time = self.get_clock().now()
            for elevator_id, idx_box_xywh in list(self.elevator_bboxes.items()):
                box_xywh = idx_box_xywh[1]
                if elevator_id not in detected_elevator_ids:
                    last_seen = self.elevator_last_seen.get(elevator_id)
                    if last_seen and (current_time - last_seen).nanoseconds < 30 * 1e9:  # 30 seconds tolerance
                        # the last seen time is in valid time limit
                        depth_buffer = self.elevator_depth_buffers[elevator_id]
                        cam_idx, cam_id, img_w, img_h, htheta_rad, vtheta_rad, hfov_rad, vfov_rad = self.elevator_bboxes_info[elevator_id]

                        if self.is_door_open(list_np_depth_imgs[cam_idx], box_xywh, depth_buffer, depth_change_threshold=0.2):
                            obj_status = ObjectStatus.OPEN
                        else:
                            obj_status = ObjectStatus.CLOSED
                        self.get_logger().info(f'undetected elevators: {obj_status}')

                        # Create a detection message for the elevator that was not detected in the current frame
                        box2d = BoundingBox2D(center=Pose2D(position=Point2D(x=box_xywh[0], y=box_xywh[1]), theta=float(0.0)),
                                            size_x=box_xywh[2], size_y=box_xywh[3])
                        det_res = GDHDetection2DExt(header=dets_msg.header, bbox=box2d, 
                                                    obj_type=ObjectType.ELEVATOR_DOOR, obj_status=obj_status,
                                                    cam_id=cam_id, img_w=img_w, img_h=img_h, 
                                                    htheta=htheta_rad, vtheta=vtheta_rad, 
                                                    hfov=hfov_rad, vfov=vfov_rad
                                                    )
                        dets_msg.detections.append(det_res)
                    else:
                        # Remove elevator data if it has not been seen for a while
                        del self.elevator_bboxes[elevator_id]
                        del self.elevator_bboxes_info[elevator_id]
                        del self.elevator_depth_buffers[elevator_id]
                        del self.elevator_last_seen[elevator_id]

        if len(dets_msg.detections) > 0:
            dets_msg.errcode = dets_msg.ERR_NONE_AND_FIND_SUCCESS
        else:
            dets_msg.errcode = dets_msg.ERR_NONE_AND_FIND_FAILURE

        return dets_msg, combined_pil_img
        
    def draw_point_on_image(self, img_np, x, y):
        # Define the point properties (e.g., radius, color)
        radius = 5
        color = (255, 0, 0)  # Red color for the point (in BGR format for OpenCV)
        thickness = -1  # Fill the circle

        # Draw a circle at (x, y) with the defined radius and color
        img_with_point_np = cv2.circle(img_np, (x, y), radius, color, thickness)

        return img_with_point_np

    def start_detect_object(self, request, response):
        self.get_logger().info(f'Incoming request @ start_detect_object, {request}')
        
        if self.yolo_model is None:
            self._init_detector()   # load model from pt file

        if self.thread is None or not self.thread.is_alive():  # 스레드가 없거나 종료된 상태인지 확인
            self.detecting = False
            self.shutdown_event.clear()  # Clear shutdown event before starting
            self.thread = threading.Thread(target=self.detect_loop)
            self.thread.start()
        
        if self.yolo_model is None:
            response.success = False
            response.message = 'Cannot initialize detector.'
        elif self.thread is None or not self.thread.is_alive():  # 스레드가 없거나 종료된 상태인지 확인
            response.success = False
            response.message = 'Cannot start detecting thread.'
        else:
            if not self.detecting:
                self.detect_object_types = request.object_types
                self.detecting = True

                response.success = True
                response.message = f'Detection started with {self.detect_object_types}.'
            else:
                response.success = False
                response.message = 'Detection is already running.'
        
        self.get_logger().info(f'                                       response: {response}')
            
        return response

    def stop_detect_object(self, request, response):
        self.get_logger().info(f'Incoming request @ stop_detect_object, {request}')
        if self.detecting:
            self.detecting = False
            response.success = True
            response.message = 'Detection stopped.'
        else:
            response.success = False
            response.message = 'Detection is not running.'

        self.get_logger().info(f'                                       response: {response}')
            
        return response
        
    def detect_loop(self):
        sleep_duration = 0.01  # 100Hz를 위한 0.01초 슬립

        try:
            while not self.shutdown_event.is_set():
                loop_start = time.time()
                
                # 1) ERP → Rectified
                t0 = time.time()
                res, list_imgs = self.get_rectified_ricoh_images(
                    htheta_list=self.htheta_list, 
                    vtheta_list=self.vtheta_list,
                    hfov=self.hfov, 
                    vfov=self.vfov
                )
                t1 = time.time()
                
                det_msg_publish_time = 0.0
                ros_img_convert_time = 0.0
                processing_time = 0.0
                
                if self.detecting:
                    self.get_logger().debug('Detecting loop start')
                    
                    # init. dets_msg
                    dets_msg = GDHDetections()
                    header = Header()
                    header.stamp = self.get_clock().now().to_msg()  # 현재 시간
                    header.frame_id = "none"
                    dets_msg.header = header
                    dets_msg.detections = []
                    
                    if res:
                        # 2) 후처리 & 탐지
                        t2 = time.time()
                        
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

                        dets_msg, combined_np_img = self.detect_common_and_draw_gp(
                            dets_msg, list_imgs, list_img_infos
                        )
                        t3 = time.time()
                        
                        # 3) filtering the detection results
                        if self.detect_object_types == self.all_object_type_id:
                            detections_filtered = dets_msg.detections
                        else:
                            detections_filtered = [
                                item for item in dets_msg.detections 
                                if int(item.obj_type) in [self.detect_object_types] and int(item.obj_type) != 254
                            ]
                        dets_msg.detections = detections_filtered

                        # 4) errcode 설정
                        if len(dets_msg.detections) > 0:
                            dets_msg.errcode = dets_msg.ERR_NONE_AND_FIND_SUCCESS
                        else:
                            dets_msg.errcode = dets_msg.ERR_NONE_AND_FIND_FAILURE
                        t4_1 = time.time()
                    else:
                        # 이미지를 받지 못한 경우
                        self.get_logger().info('No image for detection')
                        dets_msg.errcode = dets_msg.ERR_NO_IMAGE
                        
                        # combined_np_img = np.zeros((10, 10, 3), dtype=np.uint8)
                        combined_np_img = np.ones((480, 640, 3), dtype=np.uint8) * 255
                        t3 = t4_1 = time.time()

                    if self.debug_display_yolo:
                        # cv2.putText(empty_img, 'No image', (50, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                        cv2.imshow('Detection Result', combined_np_img)
                        cv2.waitKey(1)
                        
                    t4_2 = time.time()

                    if self.detecting:		# check one more time before publishing message
                        t5 = time.time()
                        # 결과를 Image 메시지로 변환하여 퍼블리시       
                        self.get_logger().debug('Convert np to ros image')    
                        dets_ros_img = self.numpy_to_ros_image(combined_np_img)
                        t6 = time.time()
                        
                        # set heartbeats error code and publish
                        self.heartbeats_det_errcode = dets_msg.errcode
                    
                        self.get_logger().info('Publish dets_msg and dets_ros_img')
                        self.get_logger().info(f'                      {dets_msg}')
                        self.publisher_detect.publish(dets_msg)
                        self.publisher_detect_img.publish(dets_ros_img)
                        t7 = time.time()
                        
                        # 기록
                        durations = {
                            'erp_to_rect': (t1 - t0) * 1000,
                            'detection': (t3 - t2) * 1000,
                            'filtering_detection': (t4_1 - t3) * 1000,
                            'imshow': (t4_2 - t4_1) * 1000,
                            'ros_convert': (t6 - t5) * 1000,
                            'publish': (t7 - t6) * 1000,
                            'total_loop': (t7 - loop_start) * 1000
                        }
                        self.get_logger().info(
                            f"Timing (ms): ERP_RECT={durations['erp_to_rect']:.1f}, "
                            f"DETECT={durations['detection']:.1f}, "
                            f"FILTER={durations['filtering_detection']:.1f}, "
                            f"IMSHOW={durations['imshow']:.1f}, "
                            f"ROS_CONV={durations['ros_convert']:.1f}, "
                            f"PUBLISH={durations['publish']:.1f}, "
                            f"TOTAL={durations['total_loop']:.1f}"
                        )
                else:
                    # self.get_logger().info('Passing detect_loop')
                    if self.debug_display_yolo:
                        if res:
                            black_np_img = self.combine_numpy_images(list_imgs)
                        else:
                            # cv2.destroyAllWindows()
                            black_np_img = np.zeros((480, 640, 3), dtype=np.uint8)
                            
                        # self.get_logger().info(f'Passing detect_loop, black_np_img: {black_np_img.shape}')
                        cv2.imshow('Detection Result', black_np_img)
                        cv2.waitKey(1)

                # self.get_logger().info(f'=======================================================')
                        
                # 종료 이벤트 또는 슬립 지속 시간을 대기
                # self.get_logger().info('Sleeping for %s seconds' % sleep_duration)
                self.shutdown_event.wait(timeout=sleep_duration)
        except Exception as e:
            # 루프 내에서 발생하는 예외를 로그로 남김
            self.get_logger().error(f"Exception in detect_loop: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
        finally:
            # 스레드 종료 시 상태 리셋
            self.get_logger().info('Exiting detect_loop')
            if self.debug_display_yolo:
                cv2.destroyAllWindows()
            self.detecting = False
        
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



    # from transformers import pipeline
    # from PIL import Image
    # import requests

    # # load pipe
    # pipe = pipeline(task="depth-estimation", model="depth-anything/Depth-Anything-V2-Small-hf")

    # # load image
    # url = 'http://images.cocodataset.org/val2017/000000039769.jpg'
    # image = Image.open(requests.get(url, stream=True).raw)

    # # inference
    # depth = pipe(image)["depth"]

    # print(depth)

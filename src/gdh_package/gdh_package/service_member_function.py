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

# Heartbeat
from gd_ifc_pkg.msg import GDHStatus
from std_msgs.msg import Header

class GDHService(Node):
    def __init__(self):
        super().__init__('gdh_service')     # node_name

        # subscription of ricoh image
        qos_profile = QoSProfile(depth=10)
        
        # CompressedImage        
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT     
        self.subscription = self.create_subscription(
            CompressedImage, '/theta/image_raw/compressed',
            self.listener_callback_ricoh_comprssed,
            qos_profile=qos_profile)
        
        # # Image
        # self.subscription = self.create_subscription(
        #     Image, '/theta/image_raw/compressed',
        #     self.listener_callback_ricoh_raw,
        #     qos_profile=qos_profile)
        
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
        self.shutdown_event = threading.Event()  # Event to signal shutdown
        
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
        self.heartbeats_det_errcode = GDHStatus.ERR_NONE  # 오류 없음
        self.publisher_status = self.create_publisher(GDHStatus, '/GDH_status', 1)
        self.timer = self.create_timer(timer_period_sec=1.0, callback=self.timer_callback)

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
            status_msg.errcode = status_msg.ERR_NONE  # 오류 없음

        self.publisher_status.publish(status_msg)
        self.get_logger().info(f"Publishing GDH status: timestamp={status_msg.header.stamp.sec}, errcode={status_msg.errcode}")

    # image
    def listener_callback_ricoh_comprssed(self, msg):
        # compressed image to numpy
        np_arr = np.frombuffer(msg.data, np.uint8)        
        self.latest_ricoh_erp_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)    # nparray is returned
        # self.get_logger().info(f'listener_callback: msg latest_ricoh_erp_image size {self.latest_ricoh_erp_image.shape}')
    
    def listener_callback_ricoh_raw(self, msg):
    	# raw image w/o compression to numpy
        self.latest_ricoh_erp_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')    # nparray is returned

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

    def detect_common(self, dets_msg, list_imgs, list_img_infos):
        dets_np_img = []

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
                    det_res = GDHDetection2DExt(header=dets_msg.header, bbox=box2d, 
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
    

    def detect_common_and_draw_gp(self, dets_msg, list_imgs, list_img_infos):
        dets_np_img = []

        # for testing
        self.get_point_client = None
        self.service_available = False  # 서비스 사용 가능 여부를 추적

        # run yolo
        results = self.yolo_model.predict(source=list_imgs)

        # Process results list
        for idx, result in enumerate(results):
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
                    det_res = GDHDetection2DExt(header=dets_msg.header, bbox=box2d, 
                                                obj_type=obj_type, obj_status=obj_status,
                                                cam_id=cam_id, img_w=img_w, img_h=img_h, 
                                                htheta=htheta_rad, vtheta=vtheta_rad, 
                                                hfov=hfov_rad, vfov=vfov_rad)
                    dets_msg.detections.append(det_res)

            # save as numpy
            np_result = result.plot()

            # 서비스 클라이언트가 아직 생성되지 않았다면 생성
            if self.get_point_client is None:
                self.get_point_client = self.create_client(GDGGetImageGuidancePoint, '/get_image_gp')
                try:
                    self.get_logger().info('Trying to create_client with srv name, GDG - get_image_gp')
                    if not self.get_point_client.wait_for_service(timeout_sec=1.0):
                        self.get_logger().error('Service not available!')
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
        if self.yolo_model is None:
            self._init_detector()
        
        if self.yolo_model is None:
            response.success = False
            response.message = 'Cannot initialize detector.'
        else:
            if not self.detecting:
                self.detect_object_types = request.object_types
                self.detecting = True
                self.shutdown_event.clear()  # Clear shutdown event before starting
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
            self.shutdown_event.set()  # Signal the thread to exit
            if self.thread is not None:
                self.thread.join(timeout=5)  # Wait for the thread to finish
                if self.thread.is_alive():
                    self.get_logger().warn('Detection thread did not exit in time.')
            response.success = True
            response.message = 'Detection stopped.'
        else:
            response.success = False
            response.message = 'Detection is not running.'

        return response
    
    
    def detect_loop(self):
        sleep_duration = 0.01  # 100Hz를 위한 0.01초 슬립
        target_object_types = self.detect_object_types

        try:
            while self.detecting and not self.shutdown_event.is_set():
                self.get_logger().debug('Detecting loop start')
                # dets_msg
                dets_msg = GDHDetections()
                header = Header()
                header.stamp = self.get_clock().now().to_msg()  # 현재 시간
                header.frame_id = "none"
                dets_msg.header = header
                dets_msg.detections = []
                
                # get image from predefined cameras
                res, list_imgs = self.get_rectified_ricoh_images(
                    htheta_list=self.htheta_list, 
                    vtheta_list=self.vtheta_list,
                    hfov=self.hfov, 
                    vfov=self.vfov
                )
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

                    dets_msg, combined_np_img = self.detect_common_and_draw_gp(
                        dets_msg, list_imgs, list_img_infos
                    )

                    if target_object_types == self.all_object_type_id:
                        detections_filtered = dets_msg.detections
                    else:
                        detections_filtered = [
                            item for item in dets_msg.detections 
                            if int(item.obj_type) in [target_object_types]
                        ]
                        
                    dets_msg.detections = detections_filtered

                    if len(dets_msg.detections) > 0:
                        dets_msg.errcode = dets_msg.ERR_NONE_AND_FIND_SUCCESS
                    else:
                        dets_msg.errcode = dets_msg.ERR_NONE_AND_FIND_FAILURE
                
                    self.get_logger().info(
                        'Set dets_msg\n\terrcode: %d,  %d(UNKNOWN), %d(FIND_FAIL), %d(FIND_SUCCESS)' % 
                        (
                            dets_msg.errcode, 
                            dets_msg.ERR_UNKNOWN, 
                            dets_msg.ERR_NONE_AND_FIND_FAILURE, 
                            dets_msg.ERR_NONE_AND_FIND_SUCCESS
                        )
                    )
                else:
                    self.get_logger().info('No image for detection')
                    dets_msg.errcode = dets_msg.ERR_NO_IMAGE
                    combined_np_img = np.zeros((10, 10, 3), dtype=np.uint8)

                # set heartbeats error code
                self.heartbeats_det_errcode = dets_msg.errcode
                
                # 결과를 Image 메시지로 변환하여 퍼블리시       
                self.get_logger().debug('Convert np to ros image')    
                dets_ros_img = self.numpy_to_ros_image(combined_np_img)
                
                self.get_logger().debug('Publish dets_msg and dets_ros_img')
                self.publisher_detect.publish(dets_msg)
                self.publisher_detect_img.publish(dets_ros_img)

                self.get_logger().debug('Sleeping for %s seconds' % sleep_duration)
                # 종료 이벤트 또는 슬립 지속 시간을 대기
                self.shutdown_event.wait(timeout=sleep_duration)
        except Exception as e:
            # 루프 내에서 발생하는 예외를 로그로 남김
            self.get_logger().error(f"Exception in detect_loop: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
        finally:
            # 스레드 종료 시 상태 리셋
            self.get_logger().info('Exiting detect_loop')
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

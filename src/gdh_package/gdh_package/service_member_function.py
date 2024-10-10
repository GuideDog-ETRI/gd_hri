from gd_ifc_pkg.srv import GDHInitializeDetectStaticObject, GDHTerminateDetectStaticObject
from gd_ifc_pkg.srv import GDHStartDetectObject, GDHStopDetectObject
from gd_ifc_pkg.srv import GDHExplainPathGP
from gd_ifc_pkg.msg import GDHDetection2DExt, GDHDetections

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

import os
from PIL import Image as PilImage
import numpy as np

# Ricoh Theta Z
import py360convert

# Yolo and detector
from ultralytics import YOLO
from vision_msgs.msg import BoundingBox2D, Pose2D, Point2D
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import threading

# # TTS
# from openai import OpenAI
# from pathlib import Path
# import pygame

# # STT
# import grpc
# import time
# import traceback
# from io import DEFAULT_BUFFER_SIZE
# import pyaudio
# import gdh_package.vito_stt_client_pb2 as pb
# import gdh_package.vito_stt_client_pb2_grpc as pb_grpc
# from requests import Session

# Heartbeat
from gd_ifc_pkg.msg import GDHStatus
from std_msgs.msg import Header
from rclpy.clock import Clock

# # TTS
# from key_wallet import OPENAI_API_KEY
# os.environ["OPENAI_API_KEY"] = OPENAI_API_KEY

# # STT
# STT_API_BASE = "https://openapi.vito.ai"
# STT_GRPC_SERVER_URL = "grpc-openapi.vito.ai:443"
# STT_CLIENT_ID = "c6BLbHh67Jwu09pZmaNR"
# STT_CLIENT_SECRET = "Kc41fr7IKwBEaQt_Jk0I21ENnN-vjU0kd7Eo67Vz"  # Key for STT
# STT_SAMPLE_RATE = 8000
# STT_ENCODING = pb.DecoderConfig.AudioEncoding.LINEAR16

# class SpeechToTextClient:
#     def __init__(self, client_id, client_secret):
#         self.client_id = client_id
#         self.client_secret = client_secret
#         self._sess = Session()
#         self._token = None

#     @property
#     def token(self):
#         if self._token is None or self._token["expire_at"] < time.time():
#             resp = self._sess.post(
#                 STT_API_BASE + "/v1/authenticate",
#                 data={"client_id": self.client_id, "client_secret": self.client_secret},
#             )
#             resp.raise_for_status()
#             self._token = resp.json()
#         return self._token["access_token"]

#     def transcribe_streaming_grpc(self, config):
#         audio = pyaudio.PyAudio()
#         stream = audio.open(format=pyaudio.paInt16,
#                             channels=1,
#                             rate=STT_SAMPLE_RATE,
#                             input=True,
#                             frames_per_buffer=DEFAULT_BUFFER_SIZE)
#         try:
#             with grpc.secure_channel(STT_GRPC_SERVER_URL, credentials=grpc.ssl_channel_credentials()) as channel:
#                 stub = pb_grpc.OnlineDecoderStub(channel)
#                 cred = grpc.access_token_call_credentials(self.token)

#                 def req_iterator(audio_stream):
#                     yield pb.DecoderRequest(streaming_config=config)
#                     while True:
#                         buff = audio_stream.read(DEFAULT_BUFFER_SIZE)
#                         if len(buff) == 0:
#                             break
#                         yield pb.DecoderRequest(audio_content=buff)

#                 print("Start speaking...")
#                 req_iter = req_iterator(stream)
#                 resp_iter = stub.Decode(req_iter, credentials=cred)

#                 for resp in resp_iter:
#                     for res in resp.results:
#                         if res.is_final:  # final speech recognition result, EPD
#                             stt_res = res.alternatives[0].text
#                             print("[stt] res: {}".format(stt_res))
#                             return stt_res
#         finally:
#             stream.stop_stream()
#             stream.close()
#             audio.terminate()



class GDHService(Node):
    def __init__(self):
        super().__init__('gdh_service')     # node_name

        # subscription of ricoh image
        qos_profile = QoSProfile(depth=10)        
        self.topic_name = '/image_raw'
        self.subscription = self.create_subscription(
            Image,
            self.topic_name,
            self.listener_callback_ricoh,
            qos_profile=qos_profile)
        self.bridge = CvBridge()
        self.subscription  # prevent unused variable warning
        self.latest_ricoh_erp_image = None

        # TODO: subscription of gopro
        # self.topic_name = '/gopro'

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
        
        self.htheta_list = [-90, 0, 90, 180]  # [-180, 180] in degree
        self.vtheta_list = [90, 90, 90, 90]     
        self.hfov = 90
        self.vfov = 70
        
        self.all_object_type_id = 255
        self.detect_object_types = self.all_object_type_id

        self.yolo_model = None
        self.yolo_conf_threshold = 0.5

        self.cam_id_ricoh = 'theta_cart'

        # self.index_to_cam_id = {
        #     0: 'theta_cart'
        # }

        # # speak code id and TTS
        # self.speech_audio_path = Path("models/temp/GDH_speak_codeid_output.mp3")        
        # self.client_openai = OpenAI()
        # self.code_sentence_map = self.load_code_sentence_table('models/code_sentence_table.txt') 

        # # STT
        # self.cli_guide_assist = self.create_client(GDGGuideAssist, '/gdg_guide_assist')

        # while not self.cli_guide_assist.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('GDGGuideAssist service not available, waiting again...')

        # self.stt_req = GDGGuideAssist.Request()
        # self.commands = self.load_commands('models/audio_cmd_list.txt')
        # self.stt_client = SpeechToTextClient(STT_CLIENT_ID, STT_CLIENT_SECRET)
        # self.config = pb.DecoderConfig(
        #     sample_rate=STT_SAMPLE_RATE,
        #     encoding=STT_ENCODING,
        #     use_itn=True,
        #     use_disfluency_filter=False,
        #     use_profanity_filter=False,
        # )
        # self.send_request()

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


    # # STT
    # def load_commands(self, path):
    #     with open(path, 'r') as file:
    #         cmd_list = [line.strip() for line in file.readlines()]
    #         self.get_logger().info(f'Loaded command list from {path}: {cmd_list}')
            
    #         return cmd_list

    # def send_request(self):
    #     while rclpy.ok():       # if connected to STT server
    #         try:
    #             self.stt_req.cmd = 255
    #             self.stt_req.type = 255

    #             stt_result = self.stt_client.transcribe_streaming_grpc(self.config)

    #             import pdb
    #             pdb.set_trace()

    #             if stt_result in self.commands:
    #                 if stt_result == '가':
    #                     self.stt_req.cmd = self.stt_req.GO_FORWARD
    #                 elif '찾아' in stt_result:
    #                     self.stt_req.cmd = self.stt_req.GOTO_OBJECT

    #                     if '문' in stt_result:
    #                         self.stt_req.type = self.stt_req.DOOR
    #                     elif '엘리베이터' in stt_result:
    #                         self.stt_req.type = self.stt_req.ELEVATOR_DOOR
    #                     elif '계단' in stt_result:
    #                         self.stt_req.type = self.stt_req.STAIRS
    #                     elif '에스컬레이터' in stt_result:
    #                         self.stt_req.type = self.stt_req.ESCALATOR
    #                     elif '횡단보도' in stt_result:
    #                         self.stt_req.type = self.stt_req.CROSSWALK
    #                     elif '지하철입구' in stt_result:
    #                         self.stt_req.type = self.stt_req.SUBWAY_GATE
    #                     elif '지하철게이트' in stt_result:
    #                         self.stt_req.type = self.stt_req.SUBWAY_TICKET_GATE
    #                     elif '스크린도어' in stt_result:
    #                         self.stt_req.type = self.stt_req.SUBWAY_SCREEN_DOOR

    #                 self.future = self.cli.call_async(self.req)
                    
    #                 while rclpy.ok():
    #                     rclpy.spin_once(self)
    #                     if self.future.done():
    #                         try:
    #                             response = self.future.result()
    #                         except Exception as e:
    #                             self.get_logger().info(f'Service call failed: {e}')
    #                         else:
    #                             if response.success:
    #                                 self.get_logger().info(f'Received response: {response.message}')
    #                             else:
    #                                 self.get_logger().info(f'No matching command found.')
    #                         break
    #             else:
    #                 self.get_logger().info(f'No matched command for: {stt_result}')
    #         except grpc.RpcError as e:
    #             self.get_logger().error(f'gRPC error: {e}')
    #         except Exception as e:
    #             self.get_logger().error(f'Error during STT processing: {e}')
    #             self.get_logger().error(traceback.format_exc())

    # # TTS
    # def play_audio(self, filepath):
    #     pygame.mixer.init()
    #     pygame.mixer.music.load(filepath)
    #     pygame.mixer.music.play()
    #     while pygame.mixer.music.get_busy():
    #         continue

    # def generate_speech(self, input_msg, model="tts-1", voice="alloy"):
    #     response = self.client_openai.audio.speech.create(
    #         model=model,
    #         voice=voice,
    #         input=input_msg,
    #     )
    #     response.stream_to_file(str(self.speech_audio_path))
    #     self.play_audio(self.speech_audio_path)


    # def speak_codeid(self, request, response):
    #     self.get_logger().info('Generating speech...')
        
    #     code = request.code_id
    #     input_msg = self.code_sentence_map.get(code, None)

    #     if input_msg is None:
    #         response.errcode = response.ERR_UNKNOWN
    #     else:
    #         self.generate_speech(input_msg)
    #         response.errcode = response.ERR_NONE
            
    #     self.get_logger().info('Incoming request @ speak_codeid\n\tresponse: %d' % (response.errcode))
    #     self.get_logger().info(f'\tcode_id: {code}, input_msg: {input_msg}')
        
    #     return response


    # def load_code_sentence_table(self, file_path):
    #     code_sentence_map = {}
    #     with open(file_path, 'r', encoding='utf-8') as file:
    #         for line in file:
    #             if ': ' in line:
    #                 code, sentence = line.strip().split(': ', 1)
    #                 code_sentence_map[int(code)] = sentence
    #     return code_sentence_map

    # image
    def listener_callback_ricoh(self, msg):
        self.latest_ricoh_erp_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')    # nparray is returned
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
            htheta = list_img_infos[idx]['htheta']
            vtheta = list_img_infos[idx]['vtheta']
            hfov = list_img_infos[idx]['hfov']
            vfov = list_img_infos[idx]['vfov']


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
                                                htheta=htheta, vtheta=vtheta, hfov=hfov, vfov=vfov
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


    # def detect_target(self, request, response):
    #     target_object_types = request.object_types

    #     if self.yolo_model is None:
    #         response.errcode = response.ERR_NOT_INIT_MODEL
    #     else:
    #         ret_img, list_imgs = self.get_image(0, theta_list=self.theta_list, hfov_list=self.hfov_list)

    #         list_img_infos = {
    #             'cam_id': [0] * len(self.theta_list),
    #             'theta': self.theta_list,
    #             'hfov': self.hfov_list
    #         }

    #         if ret_img is False:
    #             response.errcode = response.ERR_NO_IMAGE
    #         else:
    #             response = self.detect_common(list_imgs, response, list_img_infos)

    #             detections_filtered = [item for item in response.detections 
    #                                 if int(item.obj_type) in target_object_types]
                
    #             response.detections = detections_filtered

    #             if len(response.detections) > 0:
    #                 response.errcode = response.ERR_NONE_AND_FIND_SUCCESS
    #             else:
    #                 response.errcode = response.ERR_NONE_AND_FIND_FAILURE

    #     self.get_logger().info('Incoming request @ detect_target\n\tresponse: %d,  %d(UNKNOWN), %d(FIND_FAIL), %d(FIND_SUCCESS)' % 
    #                            (response.errcode, response.ERR_UNKNOWN, response.ERR_NONE_AND_FIND_FAILURE, response.ERR_NONE_AND_FIND_SUCCESS))

    #     return response
    
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

            if res:
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


            rate.sleep()
        

    # def detect(self, request, response):
    #     if self.yolo_model is None:
    #         response.errcode = response.ERR_NOT_INIT_MODEL
    #     else:
    #         theta_list = [request.theta]
    #         hfov_list = [request.hfov]

    #         ret_img, list_imgs = self.get_image(0, theta_list=theta_list, hfov_list=hfov_list)

    #         list_img_infos = {
    #             'cam_id': [0] * len(theta_list),
    #             'theta': theta_list,
    #             'hfov': hfov_list
    #         }

    #         if ret_img is False:
    #             response.errcode = response.NO_IMAGE
    #         else:
    #             response = self.detect_common(list_imgs, response, list_img_infos)
              
    #     self.get_logger().info('Incoming request @ detect\n\tresponse: %d,  %d(UNKNOWN), %d(NONE_AND_FAIL), %d(NONE_AND_FIND)' % 
    #                            (response.errcode, response.ERR_UNKNOWN, response.ERR_NONE_AND_FIND_FAILURE, response.ERR_NONE_AND_FIND_SUCCESS))

    #     return response
    

    # def detect_all(self, request, response):
    #     if self.yolo_model is None:
    #         response.errcode = response.ERR_NOT_INIT_MODEL
    #     else:
    #         ret_img0, list_imgs0 = self.get_image(0, theta_list=self.theta_list, hfov_list=self.hfov_list)  # ricoh
    #         # ret_img1, list_imgs1 = self.get_image(1)    # web or gopro

    #         # ret_img = ret_img0 * ret_img1
    #         # list_imgs = list_imgs0 + list_imgs1

    #         ret_img = ret_img0
    #         list_imgs = list_imgs0

    #         list_img_infos = {
    #             'cam_id': [0] * len(self.theta_list),
    #             'theta': self.theta_list,
    #             'hfov': self.hfov_list
    #         }

    #         if ret_img is False:
    #             response.errcode = response.NO_IMAGE
    #         else:
    #             response = self.detect_common(list_imgs, response, list_img_infos)
              
    #     self.get_logger().info('Incoming request @ detect_all\n\tresponse: %d,  %d(UNKNOWN), %d(NONE_AND_FAIL), %d(NONE_AND_FIND)' % 
    #                            (response.errcode, response.ERR_UNKNOWN, response.ERR_NONE_AND_FIND_FAILURE, response.ERR_NONE_AND_FIND_SUCCESS))

    #     return response
    

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
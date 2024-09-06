from gd_ifc_pkg.srv import GDHInitializeDetectStaticObject, GDHTerminateDetectStaticObject
from gd_ifc_pkg.srv import GDHDetectStaticObjectAll, GDHDetectStaticTargetObject, GDHDetectStaticObject
from gd_ifc_pkg.srv import GDHExplainPathGP
from gd_ifc_pkg.msg import GDHDetection2DExt
from gd_ifc_pkg.srv import GDHSpeakCodeID
from gd_ifc_pkg.srv import GDGGuideAssist

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

# TTS
from openai import OpenAI
from pathlib import Path
import pygame

# STT
import grpc
import time
import traceback
from io import DEFAULT_BUFFER_SIZE
import pyaudio
import gdh_package.vito_stt_client_pb2 as pb
import gdh_package.vito_stt_client_pb2_grpc as pb_grpc
from requests import Session

# Heartbeat
from gd_ifc_pkg.msg import GDHStatus
from std_msgs.msg import Header
from rclpy.clock import Clock

# TTS
os.environ["OPENAI_API_KEY"] = "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"  # OpenAI API for GDH ROS

# STT
STT_API_BASE = "https://openapi.vito.ai"
STT_GRPC_SERVER_URL = "grpc-openapi.vito.ai:443"
STT_CLIENT_ID = "c6BLbHh67Jwu09pZmaNR"
STT_CLIENT_SECRET = "Kc41fr7IKwBEaQt_Jk0I21ENnN-vjU0kd7Eo67Vz"  # Key for STT
STT_SAMPLE_RATE = 8000
STT_ENCODING = pb.DecoderConfig.AudioEncoding.LINEAR16

class SpeechToTextClient:
    def __init__(self, client_id, client_secret):
        self.client_id = client_id
        self.client_secret = client_secret
        self._sess = Session()
        self._token = None

    @property
    def token(self):
        if self._token is None or self._token["expire_at"] < time.time():
            resp = self._sess.post(
                STT_API_BASE + "/v1/authenticate",
                data={"client_id": self.client_id, "client_secret": self.client_secret},
            )
            resp.raise_for_status()
            self._token = resp.json()
        return self._token["access_token"]

    def transcribe_streaming_grpc(self, config):
        audio = pyaudio.PyAudio()
        stream = audio.open(format=pyaudio.paInt16,
                            channels=1,
                            rate=STT_SAMPLE_RATE,
                            input=True,
                            frames_per_buffer=DEFAULT_BUFFER_SIZE)
        try:
            with grpc.secure_channel(STT_GRPC_SERVER_URL, credentials=grpc.ssl_channel_credentials()) as channel:
                stub = pb_grpc.OnlineDecoderStub(channel)
                cred = grpc.access_token_call_credentials(self.token)

                def req_iterator(audio_stream):
                    yield pb.DecoderRequest(streaming_config=config)
                    while True:
                        buff = audio_stream.read(DEFAULT_BUFFER_SIZE)
                        if len(buff) == 0:
                            break
                        yield pb.DecoderRequest(audio_content=buff)

                print("Start speaking...")
                req_iter = req_iterator(stream)
                resp_iter = stub.Decode(req_iter, credentials=cred)

                for resp in resp_iter:
                    for res in resp.results:
                        if res.is_final:  # final speech recognition result, EPD
                            stt_res = res.alternatives[0].text
                            print("[stt] res: {}".format(stt_res))
                            return stt_res
        finally:
            stream.stop_stream()
            stream.close()
            audio.terminate()



class GDHService(Node):
    def __init__(self):
        super().__init__('gdh_service')     # node_name

        qos_profile = QoSProfile(depth=10)
        # self.topic_name = '/gopro'
        self.topic_name = '/image_raw'
        self.subscription = self.create_subscription(
            Image,
            self.topic_name,
            self.listener_callback_photo,
            qos_profile=qos_profile)
        self.bridge = CvBridge()
        self.subscription  # prevent unused variable warning
        self.latest_image = None

        # service type(in/out params), name, callback func.
        self.srv_init_detector = self.create_service(GDHInitializeDetectStaticObject, '/GDH_init_detector', self.init_detector)
        self.srv_term_detector = self.create_service(GDHTerminateDetectStaticObject, '/GDH_term_detector', self.term_detector)
        self.srv_detect_all = self.create_service(GDHDetectStaticObjectAll, '/GDH_detect_all', self.detect_all)
        self.srv_detect = self.create_service(GDHDetectStaticObject, '/GDH_detect', self.detect)
        self.srv_detect_target = self.create_service(GDHDetectStaticTargetObject, '/GDH_detect_target', self.detect_target)
        self.srv_explain_pathgp = self.create_service(GDHExplainPathGP, '/GDH_explain_path_to_gp', self.explain_path_to_gp)
        self.srv_speak_codeid = self.create_service(GDHSpeakCodeID, '/GDH_speak_codeid', self.speak_codeid)
        
        self.theta_list = [-90, 0, 90, 180]  # [-180, 180]
        self.hfov_list = [90, 90, 90, 90]
        self.vfov = 70

        self.yolo_model = None
        self.yolo_conf_threshold = 0.5

        # speak code id and TTS
        self.speech_audio_path = Path("models/GDH_speak_codeid_output.mp3")        
        self.client_openai = OpenAI()
        self.code_sentence_map = self.load_code_sentence_table('models/code_sentence_table.txt') 

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
        self.publisher_status = self.create_publisher(GDHStatus, '/GDH_status', 1)
        self.timer = self.create_timer(timer_period_sec=1.0, callback=self.timer_callback)

    # Heartbeat
    def timer_callback(self):
        header = Header()
        header.stamp = Clock().now().to_msg()  # 현재 시간
        header.frame_id = "gdh_idle"

        # Heartbeat 메시지 생성
        status_msg = GDHStatus()
        status_msg.header = header
        status_msg.errcode = 0  # 오류 없음

        self.publisher_status.publish(status_msg)
        self.get_logger().info(f"Publishing GDH status: timestamp={status_msg.header.stamp.sec}, errcode={status_msg.errcode}")


    # STT
    def load_commands(self, path):
        with open(path, 'r') as file:
            cmd_list = [line.strip() for line in file.readlines()]
            self.get_logger().info(f'Loaded command list from {path}: {cmd_list}')
            
            return cmd_list

    def send_request(self):
        while rclpy.ok():       # if connected to STT server
            try:
                self.stt_req.cmd = 255
                self.stt_req.type = 255

                stt_result = self.stt_client.transcribe_streaming_grpc(self.config)

                import pdb
                pdb.set_trace()

                if stt_result in self.commands:
                    if stt_result == '가':
                        self.stt_req.cmd = self.stt_req.GO_FORWARD
                    elif '찾아' in stt_result:
                        self.stt_req.cmd = self.stt_req.GOTO_OBJECT

                        if '문' in stt_result:
                            self.stt_req.type = self.stt_req.DOOR
                        elif '엘리베이터' in stt_result:
                            self.stt_req.type = self.stt_req.ELEVATOR_DOOR
                        elif '계단' in stt_result:
                            self.stt_req.type = self.stt_req.STAIRS
                        elif '에스컬레이터' in stt_result:
                            self.stt_req.type = self.stt_req.ESCALATOR
                        elif '횡단보도' in stt_result:
                            self.stt_req.type = self.stt_req.CROSSWALK
                        elif '지하철입구' in stt_result:
                            self.stt_req.type = self.stt_req.SUBWAY_GATE
                        elif '지하철게이트' in stt_result:
                            self.stt_req.type = self.stt_req.SUBWAY_TICKET_GATE
                        elif '스크린도어' in stt_result:
                            self.stt_req.type = self.stt_req.SUBWAY_SCREEN_DOOR

                    self.future = self.cli.call_async(self.req)
                    
                    while rclpy.ok():
                        rclpy.spin_once(self)
                        if self.future.done():
                            try:
                                response = self.future.result()
                            except Exception as e:
                                self.get_logger().info(f'Service call failed: {e}')
                            else:
                                if response.success:
                                    self.get_logger().info(f'Received response: {response.message}')
                                else:
                                    self.get_logger().info(f'No matching command found.')
                            break
                else:
                    self.get_logger().info(f'No matched command for: {stt_result}')
            except grpc.RpcError as e:
                self.get_logger().error(f'gRPC error: {e}')
            except Exception as e:
                self.get_logger().error(f'Error during STT processing: {e}')
                self.get_logger().error(traceback.format_exc())

    # TTS
    def play_audio(self, filepath):
        pygame.mixer.init()
        pygame.mixer.music.load(filepath)
        pygame.mixer.music.play()
        while pygame.mixer.music.get_busy():
            continue

    def generate_speech(self, input_msg, model="tts-1", voice="alloy"):
        response = self.client_openai.audio.speech.create(
            model=model,
            voice=voice,
            input=input_msg,
        )
        response.stream_to_file(str(self.speech_audio_path))
        self.play_audio(self.speech_audio_path)


    def speak_codeid(self, request, response):
        self.get_logger().info('Generating speech...')
        
        code = request.code_id
        input_msg = self.code_sentence_map.get(code, None)

        if input_msg is None:
            response.errcode = response.ERR_UNKNOWN
        else:
            self.generate_speech(input_msg)
            response.errcode = response.ERR_NONE
            
        self.get_logger().info('Incoming request @ speak_codeid\n\tresponse: %d' % (response.errcode))
        self.get_logger().info(f'\tcode_id: {code}, input_msg: {input_msg}')
        
        return response


    def load_code_sentence_table(self, file_path):
        code_sentence_map = {}
        with open(file_path, 'r', encoding='utf-8') as file:
            for line in file:
                if ': ' in line:
                    code, sentence = line.strip().split(': ', 1)
                    code_sentence_map[int(code)] = sentence
        return code_sentence_map

    # image
    def listener_callback_photo(self, msg):
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')    # nparray is returned
        self.get_logger().info(f'listener_callback: msg image size {self.latest_image.shape}')

        # cv2.imshow('listener', self.latest_image)
        # cv2.waitKey(10)


    def get_image(self, cam_type=0, theta_list=0, hfov_list=90):
        ret = True
        res_imgs = []

        print('func get_image is not implemented!')

        if self.latest_image is not None:
            if cam_type == 0:
                if not isinstance(theta_list, list):
                    theta_list = [theta_list]

                if not isinstance(hfov_list, list):
                    hfov_list = [hfov_list]
            
                ricohz_image = np.copy(self.latest_image)

                for theta, hfov in zip(theta_list, hfov_list):
                    planar_image = self.convert_to_planar(ricohz_image, u_deg=theta, fov_deg=(hfov, self.vfov))
                    res_imgs.append(planar_image)
                    self.get_logger().info(f'listener_callback: planar image size {planar_image.shape}')
            else:
                # use gopro or webcam
                res_imgs.append(np.copy(self.latest_image))
        else:
            ret = False

        return ret, res_imgs


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
    

    def init_detector(self, request, response):
        # Load a model
        repo = "models/gd_demo/train958/weights/best.pt"

        if os.path.exists(repo):
            self.yolo_model = YOLO(repo)
            response.errcode = response.ERR_NONE
            self.get_logger().info(f'init_detector: yolo_model is loaded from {repo}.')
        else:
            self.yolo_model = None
            response.errcode = response.ERR_UNKNOWN
            self.get_logger().info(f'init_detector: yolo_model is not loaded. Path({repo}) is not exist.')
        
        self.get_logger().info('Incoming request @ init_detector\n\tresponse: %d,  %d(UNKNOWN), %d(NONE)' % (response.errcode, response.ERR_UNKNOWN, response.ERR_NONE))

        return response
    

    def term_detector(self, request, response):
        if self.yolo_model is not None:
            del self.yolo_model
            self.yolo_model = None
            self.get_logger().info('term_detector: yolo_model is unloaded.')

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


    def detect_common(self, list_imgs, response, list_img_infos):
        # run yolo            
        results = self.yolo_model.predict(source=list_imgs)
        response.detections = []

        # Process results list
        for idx, result in enumerate(results):
            # data = result.boxes.data
            boxes_cls = result.boxes.cls          # n_det
            boxes_conf = result.boxes.conf        # n_det
            boxes_xywh = result.boxes.xywh    # n_det x 4

            cam_id = list_img_infos['cam_id'][idx]
            theta = int(list_img_infos['theta'][idx])
            hfov = int(list_img_infos['hfov'][idx])

            for i_box in range(len(boxes_cls)):
                conf = float(boxes_conf[i_box].item())
                box_xywh = [float(item.item()) for item in boxes_xywh[i_box]]
                cls = int(boxes_cls[i_box].item())

                obj_type, obj_status = self.det_result_to_gdi_code(cls)

                if conf >= self.yolo_conf_threshold:
                    box2d = BoundingBox2D(center=Pose2D(position=Point2D(x=box_xywh[0], y=box_xywh[1]), 
                                                        theta=float(0.0)), 
                                            size_x=box_xywh[2], size_y=box_xywh[3])
                    det_res = GDHDetection2DExt(cam_id=cam_id, theta=theta, hfov=hfov, 
                                                obj_type=obj_type, obj_status=obj_status,
                                                bbox=box2d)
                    # std_msgs/Header header
                    response.detections.append(det_res)

            # result.show()  # display to screen
            result.save(filename=f'result_{cam_id}_{theta}_{hfov}.jpg')  # save to disk

        if len(response.detections) > 0:
            response.errcode = response.ERR_NONE_AND_FIND_SUCCESS
        else:
            response.errcode = response.ERR_NONE_AND_FIND_FAILURE

        return response


    def detect_target(self, request, response):
        target_object_types = request.object_types

        if self.yolo_model is None:
            response.errcode = response.ERR_NOT_INIT_MODEL
        else:
            ret_img, list_imgs = self.get_image(0, theta_list=self.theta_list, hfov_list=self.hfov_list)

            list_img_infos = {
                'cam_id': [0] * len(self.theta_list),
                'theta': self.theta_list,
                'hfov': self.hfov_list
            }

            if ret_img is False:
                response.errcode = response.ERR_NO_IMAGE
            else:
                response = self.detect_common(list_imgs, response, list_img_infos)

                detections_filtered = [item for item in response.detections 
                                    if int(item.obj_type) in target_object_types]
                
                response.detections = detections_filtered

                if len(response.detections) > 0:
                    response.errcode = response.ERR_NONE_AND_FIND_SUCCESS
                else:
                    response.errcode = response.ERR_NONE_AND_FIND_FAILURE

        self.get_logger().info('Incoming request @ detect_target\n\tresponse: %d,  %d(UNKNOWN), %d(FIND_FAIL), %d(FIND_SUCCESS)' % 
                               (response.errcode, response.ERR_UNKNOWN, response.ERR_NONE_AND_FIND_FAILURE, response.ERR_NONE_AND_FIND_SUCCESS))

        return response
    

    def detect(self, request, response):
        if self.yolo_model is None:
            response.errcode = response.ERR_NOT_INIT_MODEL
        else:
            theta_list = [request.theta]
            hfov_list = [request.hfov]

            ret_img, list_imgs = self.get_image(0, theta_list=theta_list, hfov_list=hfov_list)

            list_img_infos = {
                'cam_id': [0] * len(theta_list),
                'theta': theta_list,
                'hfov': hfov_list
            }

            if ret_img is False:
                response.errcode = response.NO_IMAGE
            else:
                response = self.detect_common(list_imgs, response, list_img_infos)
              
        self.get_logger().info('Incoming request @ detect\n\tresponse: %d,  %d(UNKNOWN), %d(NONE_AND_FAIL), %d(NONE_AND_FIND)' % 
                               (response.errcode, response.ERR_UNKNOWN, response.ERR_NONE_AND_FIND_FAILURE, response.ERR_NONE_AND_FIND_SUCCESS))

        return response
    

    def detect_all(self, request, response):
        if self.yolo_model is None:
            response.errcode = response.ERR_NOT_INIT_MODEL
        else:
            ret_img0, list_imgs0 = self.get_image(0, theta_list=self.theta_list, hfov_list=self.hfov_list)  # ricoh
            # ret_img1, list_imgs1 = self.get_image(1)    # web or gopro

            # ret_img = ret_img0 * ret_img1
            # list_imgs = list_imgs0 + list_imgs1

            ret_img = ret_img0
            list_imgs = list_imgs0

            list_img_infos = {
                'cam_id': [0] * len(self.theta_list),
                'theta': self.theta_list,
                'hfov': self.hfov_list
            }

            if ret_img is False:
                response.errcode = response.NO_IMAGE
            else:
                response = self.detect_common(list_imgs, response, list_img_infos)
              
        self.get_logger().info('Incoming request @ detect_all\n\tresponse: %d,  %d(UNKNOWN), %d(NONE_AND_FAIL), %d(NONE_AND_FIND)' % 
                               (response.errcode, response.ERR_UNKNOWN, response.ERR_NONE_AND_FIND_FAILURE, response.ERR_NONE_AND_FIND_SUCCESS))

        return response
    

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
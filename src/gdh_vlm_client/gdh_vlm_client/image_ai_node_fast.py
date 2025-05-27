#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from gd_ifc_pkg.msg import UserCommand
from gd_ifc_pkg.srv import GDHSpeakText
from rclpy.qos import QoSPresetProfiles, QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage
from gd_ifc_pkg.srv import GDGGetImageGuidancePoint

# ZMQ
import cv2
import zmq
import numpy as np

# yaml, file and other utilities
import os
import yaml
import datetime
import time
import struct

from threading import Lock

from gdh_package.erp_rectify_fast_caching import erp_to_rect

qos_profile = QoSPresetProfiles.SENSOR_DATA.value     # SENSOR_DATA: Best-Effort + depth=5(또는 10) 
# qos_profile = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1)

def resize_with_long_side(img, long_side=640):
    if img is None:
        raise ValueError(f"이미지를 불러올 수 없습니다: {image_path}")

    # 원본 크기
    h, w = img.shape[:2]

    # 리사이즈 비율 계산
    if h > w:
        ratio = long_side / h
        new_h = long_side
        new_w = int(w * ratio)
    else:
        ratio = long_side / w
        new_w = long_side
        new_h = int(h * ratio)

    # 리사이즈
    resized_img = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_AREA)

    return resized_img

class QwenImageSubscriberNode(Node):
    def __init__(self):
        super().__init__('ai_image_node')
        self.get_logger().info("QwenImageSubscriberNode started.")
        
        # Load configuration
        path_to_config = 'models/gdh_config.yaml'
        try:
            with open(path_to_config) as fid:
                conf = yaml.full_load(fid)
        except FileNotFoundError:
            self.get_logger().error(f'Configuration file not found: {path_to_config}')
            raise
        except yaml.YAMLError as e:
            self.get_logger().error(f'Failed to parse YAML config: {e}')
            raise
            
        # Forward is 180 in GDG
        self.cam_id = conf['camera']['id']
        self.htheta_list = conf['camera']['htheta_list_vlm']
        self.htheta = self.htheta_list[0]
        self.vtheta = 0
        self.hfov = conf['camera']['hfov_vlm']
        self.vfov = 70
        
        self.htheta_rad = np.deg2rad(self.htheta)
        self.vtheta_rad = np.deg2rad(self.vtheta)
        self.hfov_rad = np.deg2rad(self.hfov)
        self.vfov_rad = np.deg2rad(self.vfov)
        
        if len(self.htheta_list) != 1:
            self.get_logger().warning("\n\n------------------------------------\n\n")
            self.get_logger().warning(f"htheta is not one!. Use the first htheta for SA-VLM: {self.htheta}")
            self.get_logger().warning("\n\n------------------------------------\n\n")

        # GP값 받기위한 서비스 클라이언트 생성
        self.testing_w_GDG = conf['misc']['draw_gp_vlm']
        self.get_logger().info(f'Configuration for Drawing GP is {self.testing_w_GDG}')

        if self.testing_w_GDG:
            self.get_point_client = self.create_client(GDGGetImageGuidancePoint, '/get_image_gp')
            self.service_available = False  # 서비스 사용 가능 여부를 추적
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
        else:
            self.get_point_client = None
            self.service_available = False  # 서비스 사용 가능 여부를 추적
        
        # Initialize ZeroMQ PUB socket
        try:
            self.zmq_context = zmq.Context()
            self.socket = self.zmq_context.socket(zmq.PUB)
            self.socket.bind("tcp://*:5555")
            self.get_logger().info("[Producer] PUB socket bound to port 5555.")
        except zmq.ZMQError as e:
            self.get_logger().error(f'ZMQ initialization error: {e}')
            raise

        self.image_lock = Lock()
        self.cv_image_resized = None

        # /theta/image_raw/compressed (sensor_msgs/CompressedImage) 구독
        self.image_sub = self.create_subscription(
            CompressedImage,
            #'/image_raw',
            '/theta/image_raw/compressed',
            self.image_callback,
            qos_profile
        )

        # Subscribe to user commands
        self.subscription = self.create_subscription(
            UserCommand,
            '/GDH_user_cmd',
            self.command_callback,
            10
        )
        
        # GDHSpeakText 서비스 클라이언트 생성 (서비스 이름은 '/GDH_speak_text'로 가정)
        self.speak_client = self.create_client(GDHSpeakText, '/GDH_speak_text')


    def log_info(self, msg: str):
        # 현재 시간을 사람이 읽기 좋은 형식으로 변환
        current_time = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        # 커스텀 로그 메시지 구성 및 출력
        self.get_logger().info(f'[{current_time}] {msg}')
        
    def draw_point_on_image(self, img_np, x, y, color=(255, 0, 0)):
        # Define the point properties (e.g., radius, color)
        radius = 5
          # Red color for the point (in BGR format for OpenCV)
        thickness = -1  # Fill the circle

        # Draw a circle at (x, y) with the defined radius and color
        img_with_point_np = cv2.circle(img_np, (x, y), radius, color, thickness)

        return img_with_point_np

    def image_callback(self, msg: CompressedImage):
        """/theta/image_raw/compressed 콜백"""

        # 1) CompressedImage -> OpenCV 이미지 디코딩
        np_arr = np.frombuffer(msg.data, np.uint8)         # bytes -> numpy 배열
        cv_image_erp = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # JPG/PNG 등으로 디코딩
        cv_image = erp_to_rect(erp_image=cv_image_erp, 
                                   theta=self.htheta_rad,
                                   hfov=self.hfov_rad,
                                   vfov=self.vfov_rad
                                  )
        resized = resize_with_long_side(cv_image, 640)

        with self.image_lock:
            self.cv_image_resized = resized
        
        # 2) 디코딩된 이미지를 화면에 표시 (imshow)
        if self.cv_image_resized is not None:
            cv2.imshow("Received Resized Image", self.cv_image_resized)
            cv2.waitKey(1)  # 1ms 동안 키 입력 대기 (창 업데이트용)
        
    def command_callback(self, msg: UserCommand):
        try:    
            # Extract prompt if available
            prompt = getattr(msg, 'prompt', None)

            # Select frame based on command
            if msg.usr_cmd == UserCommand.RUN_SAVLM:
                # Make a time for processing
                # unknown_sentence = "제가 한번 상황을 확인해보겠습니다."
                unknown_sentence = "저의 보행상황 설명 AI를 통해 보행로 상황을 확인해보겠습니다."
                self.call_speak_service(unknown_sentence)
                self.get_logger().info(f'Calling_speak_service, {unknown_sentence}')
            
                with self.image_lock:
                    if self.cv_image_resized is None:
                        self.get_logger().warning("No image received yet; ignoring command.")
                        return
                    frame = self.cv_image_resized.copy()
                    
                img_h, img_w, img_c = frame.shape
                
                goal_x = 0.50
                goal_y = 0.75
                use_default_gp = True
                
                # Set GP
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
                        srv_request.cam_id = self.cam_id
                        srv_request.img_w = img_w
                        srv_request.img_h = img_h
                        srv_request.htheta = self.htheta_rad
                        srv_request.vtheta = self.vtheta_rad
                        srv_request.hfov = self.hfov_rad
                        srv_request.vfov = self.vfov_rad

                        # 서비스 호출
                        try:
                            self.get_logger().info(f'Trying to get_point_client!')
                            future_get_gp = self.get_point_client.call_async(srv_request)
                            
                            if rclpy.spin_until_future_complete(self, future_get_gp, timeout_sec=1.0):
                                response = future_get_gp.result()
                                self.get_logger().info('Received get_point_client.call !!!')
                                
                                if response.errcode == response.ERR_NONE:
                                    goal_x = float(response.x) / float(img_w)
                                    goal_y = float(response.y) / float(img_h)
                                    use_default_gp = False
                                    self.get_logger().info(f'Success to draw guidance point, {response.x (goal_x)}, {response.y (goal_y)}')
                                else:
                                    self.get_logger().warn(f'Failed to get drawable guidance point: {response.errcode}')

                            else:
                                self.get_logger().error('Cannot get_point_client.call !!!')

                        except Exception as e:
                            self.get_logger().error(f'Failed to call guidance point service: {str(e)}')

                else:
                    self.get_logger().info(f"Set frame_opened and goal xy {goal_x}, {goal_y}")
            else:
                # cannot understand, say one more time.
                # unknown_sentence = "명령을 이해하지 못했습니다. 다시한번 이야기해주세요."
                # self.get_logger().info(f'Unknown command received: {msg.usr_cmd}. Calling_speak_service, {unknown_sentence}')
                # self.call_speak_service(unknown_sentence)
                return

            # Ensure frame is valid
            if frame is None:
                self.get_logger().error('No frame available to send.')
                return
    
            # Prepare header and payload
            h, w, c = frame.shape
            timestamp = time.time()
            header = struct.pack('iiidff', h, w, c, timestamp, goal_x, goal_y)
            payload = frame.tobytes()

            # Publish via ZMQ
            prompt = f"이미지는 실내환경입니다. 이미지를 분석하여 목표 위치 ({goal_x:.4f}, {goal_y:.4f})를 기반으로 시각 장애인을 위한 간단한 도보 안내를 제공해 주세요."
            if prompt:
                self.socket.send_multipart([header, payload, prompt.encode('utf-8')])
            else:
                self.socket.send_multipart([header, payload])
                
            self.get_logger().info(f"socket.send_multipart via ZMQ.")

            status_msg = f"[Producer] Sent frame ({w}x{h}) - Goal(x,y): ({goal_x:.3f}, {goal_y:.3f})"
            if prompt:
                status_msg += f" - Prompt: {prompt}"
            self.get_logger().info(status_msg)
            
            # 2) 디코딩된 이미지를 화면에 표시 (imshow)
            if frame is not None:
                frame_disp = frame.copy()
                
                goal_x_w = int(goal_x * img_w)
                goal_y_h = int(goal_y * img_h)
                
                if use_default_gp:
                    color = (0, 0, 255)
                else:
                    color = (255, 0, 0)
                    
                frame_disp = self.draw_point_on_image(frame_disp, goal_x_w, goal_y_h, color=color)
                                            
                cv2.imshow("VLM-inserted Image", frame_disp)
                cv2.waitKey(1)  # 1ms 동안 키 입력 대기 (창 업데이트용)

        except Exception as e:
            self.get_logger().error(f'Error in command_callback: {e}')
            
            
    def call_speak_service(self, sentence: str):
        req = GDHSpeakText.Request()
        req.text = sentence

        # 서비스가 사용 가능할 때까지 기다림
        if not self.speak_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("GDHSpeakText service not available!")
            return

        future = self.speak_client.call_async(req)
        future.add_done_callback(self.speak_response_callback)
        self.get_logger().info(f"Called_speak_clinet")

    def speak_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Service response: errcode={response.errcode}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")



def main(args=None):
    rclpy.init(args=args)
    node = QwenImageSubscriberNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up ZMQ resources safely
        try:
            node.socket.close()
            node.zmq_context.term()
        except Exception as e:
            node.get_logger().error(f'Error closing ZMQ: {e}')

        # Destroy OpenCV windows if any
        try:
            cv2.destroyAllWindows()
        except Exception as e:
            node.get_logger().warn(f'cv2.destroyAllWindows failed: {e}')
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


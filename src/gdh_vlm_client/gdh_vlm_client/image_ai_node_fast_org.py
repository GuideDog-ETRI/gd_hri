#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from gd_ifc_pkg.msg import UserCommand
from gd_ifc_pkg.srv import GDHSpeakText
from rclpy.qos import QoSPresetProfiles

# ZMQ
import cv2
import zmq

# yaml, file and other utilities
import os
import yaml
import datetime
import time
import struct

qos_profile = QoSPresetProfiles.SENSOR_DATA.value


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
                self.conf = yaml.full_load(fid)
        except FileNotFoundError:
            self.get_logger().error(f'Configuration file not found: {path_to_config}')
            raise
        except yaml.YAMLError as e:
            self.get_logger().error(f'Failed to parse YAML config: {e}')
            raise

        # Initialize ZeroMQ PUB socket
        try:
            self.zmq_context = zmq.Context()
            self.socket = self.zmq_context.socket(zmq.PUB)
            self.socket.bind("tcp://*:5555")
            self.get_logger().info("[Producer] PUB socket bound to port 5555.")
        except zmq.ZMQError as e:
            self.get_logger().error(f'ZMQ initialization error: {e}')
            raise

        # Load images and validate
        self.frame_closed = cv2.imread("models/closed.jpg", cv2.IMREAD_COLOR)
        if self.frame_closed is None:
            self.get_logger().error('Failed to load image: models/closed.jpg')
            raise FileNotFoundError('Image not found: models/closed.jpg')
        else:
            self.frame_closed = resize_with_long_side(self.frame_closed, 640)

        self.frame_opened = cv2.imread("models/opened.jpg", cv2.IMREAD_COLOR)
        if self.frame_opened is None:
            self.get_logger().error('Failed to load image: models/opened.jpg')
            raise FileNotFoundError('Image not found: models/opened.jpg')
        else:
            self.frame_opened = resize_with_long_side(self.frame_opened, 640)

        # Subscribe to user commands
        self.subscription = self.create_subscription(
            UserCommand,
            '/GDH_user_cmd',
            self.image_callback,
            10
        )
        
        # GDHSpeakText 서비스 클라이언트 생성 (서비스 이름은 '/GDH_speak_text'로 가정)
        self.speak_client = self.create_client(GDHSpeakText, '/GDH_speak_text')


    def log_info(self, msg: str):
        # 현재 시간을 사람이 읽기 좋은 형식으로 변환
        current_time = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        # 커스텀 로그 메시지 구성 및 출력
        self.get_logger().info(f'[{current_time}] {msg}')
        
    def image_callback(self, msg: UserCommand):
        try:
            # Extract prompt if available
            prompt = getattr(msg, 'prompt', None)

            # Select frame based on command
            if msg.usr_cmd == UserCommand.RUN_SAVLM_STOP:
                frame = self.frame_closed
                goal_x = 0.25
                goal_y = 0.75
                self.get_logger().info(f"Set frame_closed and goal xy {goal_x}, {goal_y}")
            elif msg.usr_cmd == UserCommand.RUN_SAVLM_GO:
                frame = self.frame_opened
                goal_x = 0.50
                goal_y = 0.65
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
            if prompt:
                self.socket.send_multipart([header, payload, prompt.encode('utf-8')])
            else:
                self.socket.send_multipart([header, payload])
                
            self.get_logger().info(f"socket.send_multipart via ZMQ.")

            status_msg = f"[Producer] Sent frame ({w}x{h}) - Goal(x,y): ({goal_x:.3f}, {goal_y:.3f})"
            if prompt:
                status_msg += f" - Prompt: {prompt}"
            self.get_logger().info(status_msg)

        except Exception as e:
            self.get_logger().error(f'Error in image_callback: {e}')
            
            
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
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import base64
from openai import OpenAI

# yaml
import os
import yaml
import datetime

class QwenImageSubscriberNode(Node):
    def __init__(self):
        super().__init__('ai_image_node')
        self.get_logger().info("QwenImageSubscriberNode started.")
        
        # load from conf.yaml
        path_to_config = 'models/gdh_config.yaml'
        if os.path.exists(path_to_config):
            with open(path_to_config) as fid:
                conf = yaml.full_load(fid)
        else:
            raise AssertionError(f'No gdh_config file in {path_to_config}.')

        # Qwen (OpenAI 호환) API 설정
        openai_api_key = "EMPTY"   # 상황에 맞게 수정
        openai_api_base = conf['vlm']['server_ip_addr']  # vLLM 서버 주소
        self.vlm_model_path = conf['vlm']['model_path']
        self.vlm_system_content = conf['vlm']['system_content']
        self.vlm_user_content = conf['vlm']['user_content']

        # OpenAI 클라이언트 생성
        self.client = OpenAI(
            api_key=openai_api_key,
            base_url=openai_api_base,
        )

        # /convert_image (std_msgs/String) 구독
        self.subscription = self.create_subscription(
            String,
            '/convert_image',
            self.image_callback_token_return,
            10
        )

        # **Qwen 응답을 퍼블리시할 퍼블리셔** (std_msgs/String)
        self.result_pub = self.create_publisher(
            String,
            '/qwen_result',  # 원하는 토픽 이름
            10
        )

    def log_info(self, msg: str):
        # 현재 시간을 사람이 읽기 좋은 형식으로 변환
        current_time = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        # 커스텀 로그 메시지 구성 및 출력
        self.get_logger().info(f'[{current_time}] {msg}')
        
    def image_callback_whole_return(self, msg: String):
        """
        수신된 base64 이미지를 Qwen 모델에 전달하고,
        응답 content만 발췌하여 /qwen_result 토픽으로 퍼블리시.
        """
        base64_encoded = msg.data  # Base64 문자열
        base64_qwen = f"data:image;base64,{base64_encoded}"

        try:
            response = self.client.chat.completions.create(
                model=self.vlm_model_path,
                messages=[
                    {"role": "system", "content": self.vlm_system_content},
                    {
                        "role": "user",
                        "content": [
                            {
                                "type": "image_url",
                                "image_url": {"url": base64_qwen},
                            },
                            {
                                "type": "text",
                                "text": self.vlm_user_content
                            },
                        ],
                    },
                ],
            )

            # 1) Qwen 응답 메시지 content 추출
            response_content = response.choices[0].message.content

            # 2) 콘솔 출력 (선택)
            # self.get_logger().info(f"[Qwen content] {response_content}")

            # 3) 토픽 퍼블리시
            out_msg = String()
            out_msg.data = response_content
            self.result_pub.publish(out_msg)

            # self.get_logger().info("[Qwen content] published to /qwen_result")
            self.log_info("[Qwen content] published to /qwen_result")

        except Exception as e:
            self.get_logger().error(f"Error calling Qwen API: {e}")
            
    def image_callback_token_return(self, msg: String):
        """
        수신된 base64 이미지를 Qwen 모델에 전달하고,
        응답 content만 발췌하여 /qwen_result 토픽으로 퍼블리시.
        """
        base64_encoded = msg.data  # Base64 문자열
        base64_qwen = f"data:image;base64,{base64_encoded}"

        try:
            response = self.client.chat.completions.create(
                model=self.vlm_model_path,                
                messages=[
                    {"role": "system", "content": self.vlm_system_content},
                    {
                        "role": "user",
                        "content": [
                            {
                                "type": "image_url",
                                "image_url": {"url": base64_qwen},
                            },
                            {
                                "type": "text",
                                "text": self.vlm_user_content
                            },
                        ],
                    },
                ],
                stream=True	# 스트리밍 활성화
            )
            
            for chunk in response:
                # 1) Qwen 응답 메시지 content 추출
                chunk_content = chunk.choices[0].delta.content

                # 2) 콘솔 출력 (선택)
                # self.get_logger().info(f"[Qwen chunk content] {chunk_content}")
                self.log_info(f"[Qwen chunk content] {chunk_content}")

                # 3) 토픽 퍼블리시
                out_msg = String()
                out_msg.data = chunk_content
                self.result_pub.publish(out_msg)

                self.get_logger().info("[Qwen content] published to /qwen_result")

        except Exception as e:
            self.get_logger().error(f"Error calling Qwen API: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = QwenImageSubscriberNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

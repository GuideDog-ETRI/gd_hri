#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import re
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from gd_ifc_pkg.srv import GDHSpeakText

class QwenConsoleNode(Node):
    def __init__(self):
        super().__init__('qwen_console_node')
        # 노드 시작 시 출력(필요없으면 이 줄 삭제 가능)
        print("QwenConsoleNode started, subscribing /qwen_result")

        # /qwen_result (std_msgs/String) 구독
        self.subscription = self.create_subscription(
            String,
            '/qwen_result',
            self.callback_qwen_result,
            10
        )
        # 토큰들을 누적할 버퍼 초기화
        self.sentence_buffer = ""
        
        # GDHSpeakText 서비스 클라이언트 생성 (서비스 이름은 '/GDH_speak_text'로 가정)
        self.speak_client = self.create_client(GDHSpeakText, '/GDH_speak_text')
        
        # 문장 종결부호(. ! ?)를 포함하는 문장을 추출하는 정규식 패턴
        self.sentence_pattern = re.compile(r'^(.*?[.!?])\s*(.*)$')


    def callback_qwen_result(self, msg: String):
        # 새로 들어온 token들을 버퍼에 누적
        self.sentence_buffer += msg.data
        
        # 누적된 버퍼에서 XML 태그 제거
        cleaned_buffer = re.sub(r'<[^>]*>', '', self.sentence_buffer)
        
        # 완성된 문장이 있는지 확인하여 추출 (문장 종결부호 포함)
        while True:
            match = self.sentence_pattern.match(cleaned_buffer)
            if not match:
                break  # 더 이상 완성된 문장이 없으면 종료

            # 추출된 문장 (종결부호 포함)
            sentence = match.group(1).strip()
            # 남은 미완성 텍스트
            remainder = match.group(2)
            
            print(f"[QWEN] {sentence}\n")
            self.call_speak_service(sentence)
            
            # 남은 부분을 대상으로 다시 검사
            cleaned_buffer = remainder
        
        # 버퍼에 남은 미완성 텍스트를 저장 (추후 토큰 추가 시 다시 처리)
        self.sentence_buffer = cleaned_buffer
            
    def call_speak_service(self, sentence: str):
        req = GDHSpeakText.Request()
        req.text = sentence

        # 서비스가 사용 가능할 때까지 기다림
        if not self.speak_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("GDHSpeakText service not available!")
            return

        future = self.speak_client.call_async(req)
        future.add_done_callback(self.speak_response_callback)

    def speak_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Service response: errcode={response.errcode}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = QwenConsoleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

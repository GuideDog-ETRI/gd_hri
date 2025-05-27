# ROS
import rclpy
from rclpy.node import Node

import os
import datetime

# Melo TTS w/ OnDevice Lab
import time
import zmq
import struct

# GDH Interface
from gd_ifc_pkg.srv import GDHSpeakCodeID
from gd_ifc_pkg.srv import GDHSpeakText

class MsgToAudio(Node):
    def __init__(self):
        super().__init__('msg_to_audio')     # node_name

        # service type(in/out params), name, callback func.
        self.srv_speak_codeid = self.create_service(GDHSpeakCodeID, '/GDH_speak_codeid', self.speak_codeid)
        self.srv_speak_text = self.create_service(GDHSpeakText, '/GDH_speak_text', self.speak_text)
        
        # load speak code id
        self.code_sentence_map = self.load_code_sentence_table('models/code_sentence_table.txt') 

        # 기본 속도
        self.default_speed = 1.5
        # 서버 주소 (환경 변수로 설정 가능)
        # 'tcp://*:4070'
        self.server_addr = os.getenv('MELO_TTS_SERVER', 'tcp://127.0.0.1:4070')
        # TTS 서버 초기화 및 연결 시도
        self.init_tts(self.default_speed)
        
    def init_tts(self, speed):
        # ZMQ 컨텍스트 및 소켓 설정 for melo tts connection
        self.zmq_context = zmq.Context()
        self._create_and_connect_socket()
                
        # 고정된 문장과 속도 목록
        init_sentences = [
            ("티티에스를 초기화 합니다.", 2.0),
            ("큐를 초기화 합니다.", 2.0),
            ("1배속 속도를 테스트 하고자 합니다.", 1.0),
            ("1점5배속 속도를 테스트 하고자 합니다.", 1.5),
            ("2배속 속도를 테스트 하고자 합니다.", 2.0),
            ("티티에스 모델이 세팅 완료되었습니다.", 1.5),
        ]
                
        try:
            # 구독자가 연결될 때까지 잠시 대기
            time.sleep(1.0)
        
            for i, (text, test_speed) in enumerate(init_sentences):
                print("\n")
                print(f"🔤 {i+1}/{len(init_sentences)} 번째 문장 전송 중...")
                print(f"📝 문장: {text}")
                print(f"⚡ 속도: {speed}배속")
            
                # 속도(float)와 텍스트를 바이너리로 변환
                self.set_speech_speed(test_speed)
                self.send_speech_message(text)
            
                # 1초 대기
                time.sleep(1.0)
            
            print(f"\n👋 {len(init_sentences)}개의 문장 처리가 완료되었습니다.")
            print(f"\n Successfully initialized the Melo TTS")
        except Exception as e:
            self.get_logger().error(f"Error during TTS init testing: {e}")
            
    def _create_and_connect_socket(self):
        # 기존 소켓이 있으면 닫기
        try:
            if hasattr(self, 'socket'):
                self.socket.close()
        except Exception:
            pass
            
        # 새로운 PUB 소켓 생성 및 연결
        self.socket = self.zmq_context.socket(zmq.PUSH)  # PUSH 소켓으로 변경
        
        connected = False
        while not connected and rclpy.ok():
            try:
                self.socket.connect(self.server_addr)
                connected = True
                self.log_info(f"Connected to TTS server at {self.server_addr}")
            except zmq.ZMQError as e:
                self.get_logger().warning(
                    f"Failed to connect to TTS server at {self.server_addr}: {e}. Retrying in 1s...")
                time.sleep(1.0)

    def log_info(self, msg: str):
        # 현재 시간을 사람이 읽기 좋은 형식으로 변환
        current_time = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        # 커스텀 로그 메시지 구성 및 출력
        self.get_logger().info(f'[{current_time}] {msg}')
    
    def set_speech_speed(self, speed):
        self.speed = speed
        self.speed_bytes = struct.pack('f', speed)

    def send_speech_message(self, input_msg):
        self.get_logger().info(f'Sending_speech_message ...')
        data = self.speed_bytes + input_msg.encode('utf-8')
        
        try:
            self.socket.send(data)
            self.get_logger().info(f'Sended speech message: (speed: {self.speed}), {input_msg}')
        except zmq.ZMQError as e:
            self.get_logger().warning(f'Failed to send message: {e}. Attempting to reconnect...')
            # 서버 재연결 시도
            self._create_and_connect_socket()
            try:
                self.socket.send(data)
                self.get_logger().info(f'Resent speech message: (speed: {self.speed}), {input_msg}')
            except zmq.ZMQError as e2:
                self.get_logger().error(f'Resend failed after reconnect: {e2}')

    def speak_codeid(self, request, response):
        self.get_logger().info('Generating speech from codeID ...')
        code = request.code_id
        input_msg = self.code_sentence_map.get(code, None)
        
        if input_msg is None:
            response.errcode = response.ERR_UNKNOWN
        else:
            self.send_speech_message(input_msg)
            response.errcode = response.ERR_NONE
            
        self.get_logger().info('Incoming request @ speak_codeid\n\tresponse: %d' % (response.errcode))
        self.get_logger().info(f'\tcode_id: {code}, input_msg: {input_msg}')
        
        return response
        
    def speak_text(self, request, response):
        self.get_logger().info('Generating speech from text ...')
        
        input_msg = request.text

        if input_msg is None:
            response.errcode = response.ERR_UNKNOWN
        else:
            self.send_speech_message(input_msg)
            response.errcode = response.ERR_NONE
            
        self.get_logger().info('Incoming request @ speak_text\n\tresponse: %d' % (response.errcode))
        self.get_logger().info(f'\tinput_msg: {input_msg}')
        
        return response

    def load_code_sentence_table(self, file_path):
        code_sentence_map = {}
        with open(file_path, 'r', encoding='utf-8') as file:
            for line in file:
                if ': ' in line:
                    code, sentence = line.strip().split(': ', 1)
                    code_sentence_map[int(code)] = sentence
        return code_sentence_map
        
    def destroy_node(self):
        # 노드 종료 시 ZMQ 정리
        self.log_info('Shutting down TTS socket and context...')
        try:
            self.socket.close()
            self.zmq_context.term()
            self.log_info('TTS socket and context terminated.')
        except Exception as e:
            self.get_logger().error(f'Error terminating ZMQ: {e}')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    gdh_service_node = MsgToAudio()
    try:
        rclpy.spin(gdh_service_node)
    except KeyboardInterrupt:
        pass
    finally:
        gdh_service_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

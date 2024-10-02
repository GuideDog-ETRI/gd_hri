# ROS
import rclpy
from rclpy.node import Node

import os

# GDH Interface
from gd_ifc_pkg.srv import GDHSpeakCodeID

# TTS
from openai import OpenAI
from pathlib import Path
import pygame

from key_lib import OPENAI_API_KEY

# TTS
os.environ["OPENAI_API_KEY"] = OPENAI_API_KEY


class MsgToAudio(Node):
    def __init__(self):
        super().__init__('msg_to_audio')     # node_name

        # service type(in/out params), name, callback func.
        self.srv_speak_codeid = self.create_service(GDHSpeakCodeID, '/GDH_speak_codeid', self.speak_codeid)
        
        # speak code id and TTS
        self.speech_audio_path = Path("models/GDH_speak_codeid_output.mp3")        
        self.client_openai = OpenAI()
        self.code_sentence_map = self.load_code_sentence_table('models/code_sentence_table.txt') 

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

def main(args=None):
    rclpy.init(args=args)

    gdh_service_node = MsgToAudio()
    rclpy.spin(gdh_service_node)
    gdh_service_node.destroy_node()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
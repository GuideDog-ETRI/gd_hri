# ROS
import rclpy
from rclpy.node import Node

import os
import hashlib

# GDH Interface
from gd_ifc_pkg.srv import GDHSpeakCodeID
from gd_ifc_pkg.srv import GDHSpeakText

# TTS
from openai import OpenAI
from pathlib import Path
import pygame

from .key_wallet import OPENAI_API_KEY

# TTS
os.environ["OPENAI_API_KEY"] = OPENAI_API_KEY


class MsgToAudio(Node):
    def __init__(self):
        super().__init__('msg_to_audio')     # node_name

        # service type(in/out params), name, callback func.
        self.srv_speak_codeid = self.create_service(GDHSpeakCodeID, '/GDH_speak_codeid', self.speak_codeid)
        self.srv_speak_text = self.create_service(GDHSpeakText, '/GDH_speak_text', self.speak_text)
        
        # speak code id and TTS
        self.mp3_output_folder = 'models/temp'
        
        if not os.path.exists(self.mp3_output_folder):
            os.makedirs(self.mp3_output_folder)

        self.client_openai = OpenAI()
        self.code_sentence_map = self.load_code_sentence_table('models/code_sentence_table.txt') 

    # TTS
    def play_audio(self, filepath):
        try:
            pygame.mixer.init()
            pygame.mixer.music.load(filepath)
            pygame.mixer.music.play()
            while pygame.mixer.music.get_busy():
                continue
        except pygame.error as e:
            self.get_logger().error(f"Failed to play audio: {e}")
        finally:
            pygame.mixer.quit()  # Release device

    def generate_speech(self, input_msg, filepath, model="tts-1", voice="alloy"):
        response = self.client_openai.audio.speech.create(
            model=model,
            voice=voice,
            input=input_msg,
        )
        response.stream_to_file(filepath)
        self.play_audio(filepath)

    def speak_codeid(self, request, response):
        self.get_logger().info('Generating speech from codeID ...')
        
        code = request.code_id
        input_msg = self.code_sentence_map.get(code, None)

        if input_msg is None:
            response.errcode = response.ERR_UNKNOWN
        else:
            # Generate a hash for the input message to use as the filename
            hash_object = hashlib.md5(input_msg.encode())
            audio_filename = f"{hash_object.hexdigest()}.mp3"
            audio_filepath = Path(self.mp3_output_folder) / audio_filename

            if audio_filepath.exists():
                self.get_logger().info(f"Loading existing audio file for input_msg: {input_msg}")
                self.play_audio(audio_filepath)
            else:
                self.get_logger().info(f"Generating new audio file for input_msg: {input_msg}")
                self.generate_speech(input_msg, str(audio_filepath))
            
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
            # Generate a hash for the input message to use as the filename
            hash_object = hashlib.md5(input_msg.encode())
            audio_filename = f"{hash_object.hexdigest()}.mp3"
            audio_filepath = Path(self.mp3_output_folder) / audio_filename

            if audio_filepath.exists():
                self.get_logger().info(f"Loading existing audio file for input_msg: {input_msg}")
                self.play_audio(audio_filepath)
            else:
                self.get_logger().info(f"Generating new audio file for input_msg: {input_msg}")
                self.generate_speech(input_msg, str(audio_filepath))
            
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

def main(args=None):
    rclpy.init(args=args)

    gdh_service_node = MsgToAudio()
    rclpy.spin(gdh_service_node)
    gdh_service_node.destroy_node()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# ROS
import rclpy
from rclpy.node import Node

# GDH Interface
from gd_ifc_pkg.msg import UserCommand

# STT
import grpc
import time
import traceback
from io import DEFAULT_BUFFER_SIZE
import pyaudio
import gdh_speech_audio.vito_stt_client_pb2 as pb
import gdh_speech_audio.vito_stt_client_pb2_grpc as pb_grpc
from requests import Session
import pygame

from .key_wallet import STT_CLIENT_ID, STT_CLIENT_SECRET

# STT
STT_API_BASE = "https://openapi.vito.ai"
STT_GRPC_SERVER_URL = "grpc-openapi.vito.ai:443"
# STT_CLIENT_ID
# STT_CLIENT_SECRET
STT_SAMPLE_RATE = 8000
STT_ENCODING = pb.DecoderConfig.AudioEncoding.LINEAR16

# TTS
def play_audio(filepath):
    try:
        pygame.mixer.init()
        pygame.mixer.music.load(filepath)
        pygame.mixer.music.play()
        while pygame.mixer.music.get_busy():
            continue
    finally:
        pygame.mixer.quit()  # Release device

class SpeechToTextClient(Node):
    def __init__(self, client_id, client_secret):
        super().__init__('speech_to_text_client')  # ROS 노드 초기화
        self.client_id = client_id
        self.client_secret = client_secret
        self._sess = Session()
        self._token = None
        self.stream = None

        # PyAudio 초기화 및 스트림 열기
        self.audio = pyaudio.PyAudio()
        self.stream = None
        self.open_stream()

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

    def open_stream(self):
        """스트림이 없을 때만 새로 엽니다."""
        if self.stream is None:
            self.stream = self.audio.open(
                format=pyaudio.paInt16,
                channels=1,
                rate=STT_SAMPLE_RATE,
                input=True,
                frames_per_buffer=DEFAULT_BUFFER_SIZE
            )
            self.get_logger().info("Audio stream opened.")

    def transcribe_streaming_grpc(self, config):        
        try:
            # 스트림이 닫혔으면 다시 열기
            self.open_stream()

        
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

                req_iter = req_iterator(self.stream)
                resp_iter = stub.Decode(req_iter, credentials=cred)

                for resp in resp_iter:
                    for res in resp.results:
                        if res.is_final:  # final speech recognition result, EPD
                            stt_res = res.alternatives[0].text
                            print("[stt] res: {}".format(stt_res))
                            return stt_res
        except grpc.RpcError as e:
            play_audio('models/temp/no_net_connection.mp3')
            self.get_logger().error(f"gRPC error: {e}")
        except Exception as e:
            self.get_logger().error(f"Error in PyAudio: {e}")
        
        # 실패 시 None 반환
        return None   

    def __del__(self):
        # 노드 종료 시 한 번만 스트림과 PyAudio 정리
        if self.stream is not None:
            self.stream.stop_stream()
            self.stream.close()
        self.audio.terminate()     


class SpeechToCmd(Node):
    def __init__(self):
        super().__init__('speech_to_cmd')     # node_name

        self.commands = self.load_commands('models/audio_cmd_list.txt')
        self.publisher_cmd = self.create_publisher(UserCommand, '/GDH_user_cmd', 1)
        
        # STT
        self.stt_client = SpeechToTextClient(STT_CLIENT_ID, STT_CLIENT_SECRET)
        self.config = pb.DecoderConfig(
            sample_rate=STT_SAMPLE_RATE,
            encoding=STT_ENCODING,
            use_itn=True,
            use_disfluency_filter=False,
            use_profanity_filter=False,
            keywords=list(self.commands.keys()),
        )
        self.listen_and_pub_msg()

    # STT
    def load_commands(self, path):
        with open(path, 'r') as file:
            # cmd_list = [line.strip() for line in file.readlines()]
            # self.get_logger().info(f'Loaded command list from {path}: {cmd_list}')

            cmd_list = {}

            for line in file.readlines():
                cmd_param = line.strip()
                cmd_param_list = cmd_param.split('|')

                cmd_list[cmd_param_list[0]] = {
                    'usr_cmd': cmd_param_list[1],
                    'cmd_param': cmd_param_list[2]
                }
            self.get_logger().info(f'Loaded all info from {path}: {cmd_list}')    
            self.get_logger().info(f'Loaded command list from {path}: {cmd_list.keys()}')
            
            return cmd_list
    
    def listen_and_pub_msg(self):
        msg = UserCommand()

        self.get_logger().info(f"Start list_and_pub_msg")

        while rclpy.ok():  # Node가 실행 중일 때
            try:
                # Start recording 알림음
                play_audio('models/temp/start_recording.ogg')
                self.get_logger().info(f"Start recording and STT!")
                stt_result_w_space = self.stt_client.transcribe_streaming_grpc(self.config)
                if not stt_result_w_space:
                    self.get_logger().error("No result from STT.")
                    continue

                stt_result = stt_result_w_space.replace(" ", "")  # 공백 제거

                # Stop recording 알림음
                play_audio('models/temp/stop_recording.ogg')
                self.get_logger().info(f"Stop recording, Receiving a result from STT!: {stt_result}")

                if stt_result in self.commands.keys():
                    usr_cmd_str = self.commands[stt_result]['usr_cmd']
                    cmd_param_str = self.commands[stt_result]['cmd_param']

                    self.get_logger().info(f"Matched command: {stt_result}")
                    play_audio('models/temp/recognized.ogg')

                    try:
                        self.get_logger().info(f"Start to pub msg")
                        msg.usr_cmd = getattr(UserCommand, usr_cmd_str, UserCommand.NONE)
                        msg.cmd_param = getattr(UserCommand, cmd_param_str, UserCommand.NONE)

                        self.get_logger().info(f"Msg: {msg}")

                        self.publisher_cmd.publish(msg)
                        self.get_logger().info(f"SpeechToCommand Published: {msg.usr_cmd}, {msg.cmd_param}")
                    except AttributeError as e:
                        self.get_logger().error(f"AttributeError: {e}")
                else:
                    self.get_logger().info(f"No matched command for: {stt_result}")
            except grpc.RpcError as e:
                # play_audio('models/temp/no_net_connection.mp3')
                self.get_logger().error(f"gRPC error: {e}")
            except Exception as e:
                # play_audio('models/temp/no_net_connection.mp3')
                self.get_logger().error(f"Error during STT processing: {e}")
                self.get_logger().error(traceback.format_exc())

            self.get_logger().info(f"End of while")

        self.get_logger().info(f"End of listen_and_pub_msg")


def main(args=None):
    rclpy.init(args=args)

    gdh_publisher_node = SpeechToCmd()
    rclpy.spin(gdh_publisher_node)
    gdh_publisher_node.destroy_node()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()

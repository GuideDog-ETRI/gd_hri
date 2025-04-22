#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ------------------------------------------------------------
# ROS2 노드: SpeechToCmd  ‑‑ Whisper STT 버전
# ------------------------------------------------------------

# ──────────────────  ROS  ──────────────────
import rclpy
from rclpy.node import Node
from gd_ifc_pkg.msg import UserCommand

# ──────────────────  오디오/알림음  ──────────────────
import pygame
def play_audio(filepath: str):
    """단순 TTS/효과음 재생 (blocking)."""
    try:
        pygame.mixer.init()
        pygame.mixer.music.load(filepath)
        pygame.mixer.music.play()
        while pygame.mixer.music.get_busy():
            pass
    finally:
        pygame.mixer.quit()

# ──────────────────  Whisper STT  ──────────────────
import numpy as np
import speech_recognition as sr
import whisper, torch
from queue import Queue
from datetime import datetime, timedelta

WHISPER_MODEL_NAME = "large-v3-turbo"      # 필요하면 tiny / base / … 로 교체
MIC_SAMPLE_RATE   = 16_000            # Whisper 권장 값
ENERGY_THRESHOLD  = 1000
RECORD_TIMEOUT    = 1.0               # 콜백 오디오 길이(초)
PHRASE_TIMEOUT    = 1.0               # 침묵 지속(초) → 문장 경계

class SpeechToTextClient(Node):
    """
    Whisper STT:
    listen_in_background → Queue 수신 → phrase_timeout 침묵이면 문장 완성
    외부에선 transcribe_once() 호출 시 완성된 문장(str) 또는 None 을 받는다.
    """
    def __init__(self):
        super().__init__("speech_to_text_client")

        # Whisper 모델
        self.get_logger().info(f"Loading Whisper model “{WHISPER_MODEL_NAME}”…")
        self.model = whisper.load_model(WHISPER_MODEL_NAME)
        self.get_logger().info("Whisper loaded.")

        # SpeechRecognition 설정
        self.recognizer = sr.Recognizer()
        self.recognizer.energy_threshold = ENERGY_THRESHOLD
        self.recognizer.dynamic_energy_threshold = False

        self.mic = sr.Microphone(sample_rate=MIC_SAMPLE_RATE)
        with self.mic:
            self.recognizer.adjust_for_ambient_noise(self.mic, duration=0.5)

        # 버퍼·큐
        self.data_queue: Queue[bytes] = Queue()
        self.phrase_bytes: bytes = b""
        self.phrase_time: datetime | None = None

        # 백그라운드 녹음 스레드
        self.recognizer.listen_in_background(
            self.mic,
            self._record_callback,
            phrase_time_limit=RECORD_TIMEOUT,
        )

    # ───────── 내부 콜백 ─────────
    def _record_callback(self, _sr, audio: sr.AudioData):
        self.data_queue.put(audio.get_raw_data())

    # ───────── 외부 API ─────────
    def transcribe_once_base(self) -> str | None:
        """완성된 문장이 생길 때까지 블록, 생기면 문자열·실패면 None."""
        while rclpy.ok():
            if not self.data_queue.empty():
                now = datetime.utcnow()

                # 침묵이 길었다면 새 문장
                if self.phrase_time and (now - self.phrase_time) > timedelta(seconds=PHRASE_TIMEOUT):
                    self.phrase_bytes = b""

                self.phrase_time = now

                # 큐 비우기
                while not self.data_queue.empty():
                    self.phrase_bytes += self.data_queue.get()

                # PCM int16 → float32(-1~1)
                audio_np = (np.frombuffer(self.phrase_bytes, dtype=np.int16)
                            .astype(np.float32) / 32768.0)

                try:
                    result = self.model.transcribe(
                        audio_np,
                        fp16=torch.cuda.is_available(),
                    )
                    text = result["text"].strip()
                    if text:
                        return text
                except Exception as e:
                    self.get_logger().error(f"Whisper error: {e}")
                    return None
            else:
                rclpy.spin_once(self, timeout_sec=0.1)

        return None  # 노드가 종료될 때
    
    def transcribe_once(self) -> str | None:
        """
        완성된 문장이 끝났을 때만 Whisper STT 호출.
        - 3초 이상 침묵 지속되면 문장 경계로 간주
        - 이후 큐가 비었을 때만 Whisper 호출
        """
        while rclpy.ok():
            if not self.data_queue.empty():
                now = datetime.utcnow()
    
                # 침묵 지속 시간 체크
                if self.phrase_time and (now - self.phrase_time) > timedelta(seconds=PHRASE_TIMEOUT):
                    # 아직 큐에 오디오가 있으면 다음 루프에서 처리
                    if not self.data_queue.empty():
                        continue
    
                    # 말이 끝났고, 큐도 비어있음 → 이제 STT 시도
                    audio_np = (np.frombuffer(self.phrase_bytes, dtype=np.int16)
                                .astype(np.float32) / 32768.0)
                    self.phrase_bytes = b""  # 다음 문장을 위해 초기화
    
                    try:
                        result = self.model.transcribe(audio_np, fp16=torch.cuda.is_available())
                        text = result["text"].strip()
                        return text if text else None
                    except Exception as e:
                        self.get_logger().error(f"Whisper error: {e}")
                        return None
    
                # 문장 이어쓰기
                self.phrase_time = now
                while not self.data_queue.empty():
                    self.phrase_bytes += self.data_queue.get()
    
            else:
                rclpy.spin_once(self, timeout_sec=0.1)
    
        return None
# ──────────────────  명령 노드  ──────────────────
class SpeechToCmd(Node):
    def __init__(self):
        super().__init__("speech_to_cmd")

        self.commands = self._load_commands("models/audio_cmd_list.txt")
        self.publisher_cmd = self.create_publisher(UserCommand, "/GDH_user_cmd", 1)

        # STT
        self.stt_client = SpeechToTextClient()

        self._listen_and_publish()

    # --- 명령 리스트 로드 ---
    def _load_commands(self, path: str):
        cmd = {}
        with open(path, "r") as f:
            for line in f.readlines():
                key, usr_cmd, param = line.strip().split("|")
                cmd[key.replace(" ", "")] = {"usr_cmd": usr_cmd, "cmd_param": param}
        self.get_logger().info(f"Loaded commands: {list(cmd.keys())}")
        return cmd

    # --- 메인 루프 ---
    def _listen_and_publish(self):
        msg = UserCommand()
        self.get_logger().info("Start listen_and_publish")

        while rclpy.ok():
            try:
                play_audio("models/temp/start_recording.ogg")
                self.get_logger().info("Recording…")

                stt_result = self.stt_client.transcribe_once()
                if not stt_result:
                    self.get_logger().error("STT returned nothing.")
                    continue

                stt_key = stt_result.replace(" ", "")
                play_audio("models/temp/stop_recording.ogg")
                self.get_logger().info(f"STT result: “{stt_key}”")

                if stt_key in self.commands:
                    play_audio("models/temp/recognized.ogg")
                    info = self.commands[stt_key]

                    msg.usr_cmd   = getattr(UserCommand, info["usr_cmd"],   UserCommand.NONE)
                    msg.cmd_param = getattr(UserCommand, info["cmd_param"], UserCommand.NONE)

                    self.publisher_cmd.publish(msg)
                    self.get_logger().info(f"Published: {msg.usr_cmd}, {msg.cmd_param}")
                else:
                    self.get_logger().info("No matched command.")
            except Exception as e:
                self.get_logger().error(f"Error: {e}")

        self.get_logger().info("listen_and_publish ended")

# ──────────────────  main  ──────────────────
def main(args=None):
    rclpy.init(args=args)
    node = SpeechToCmd()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ------------------------------------------------------------
# ROS2 노드: SpeechToCmd  – Whisper STT 초저지연 버전
#   ◦ RECORD_TIMEOUT 0.25 s  → _record_callback 호출 주기 단축
#   ◦ PHRASE_TIMEOUT 0.4 s   → 400 ms 침묵 후 즉시 인식
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
import time
import queue
import numpy as np
import speech_recognition as sr
import whisper, torch
from queue import Queue

WHISPER_MODEL_NAME = "large-v3-turbo"       # 필요하면 tiny / base / … 로 교체
MIC_SAMPLE_RATE   = 16_000                  # Whisper 권장 값
ENERGY_THRESHOLD  = 1000
RECORD_TIMEOUT    = 0.25                    # 콜백 오디오 길이(초)
PHRASE_TIMEOUT    = 0.4                     # 400 ms 침묵 지속 → 문장 경계

class SpeechToTextClient(Node):
    """listen_in_background → Queue → 침묵 0.4 s 지나면 Whisper 호출."""

    def __init__(self):
        super().__init__("speech_to_text_client")

        # Whisper 모델 로드
        self.get_logger().info(f"Loading Whisper model “{WHISPER_MODEL_NAME}”…")
        self.model = whisper.load_model(WHISPER_MODEL_NAME)
        self.get_logger().info("Whisper loaded.")

        # SpeechRecognition 설정
        self.recognizer = sr.Recognizer()
        self.recognizer.energy_threshold = ENERGY_THRESHOLD
        self.recognizer.dynamic_energy_threshold = False

        self.mic = sr.Microphone(sample_rate=MIC_SAMPLE_RATE)
        with self.mic:
            self.recognizer.adjust_for_ambient_noise(self.mic, duration=0.4)

        # 버퍼·큐
        self.data_queue: Queue[bytes] = Queue()

        # 백그라운드 녹음 스레드 (250 ms chunk)
        self.recognizer.listen_in_background(
            self.mic,
            self._record_callback,
            phrase_time_limit=RECORD_TIMEOUT,
        )

    # ───────── 내부 콜백 ─────────
    def _record_callback(self, _sr, audio: sr.AudioData):
        self.data_queue.put(audio.get_raw_data())

    # ───────── 외부 API ─────────
    def transcribe_once(self) -> str | None:
        """PHRASE_TIMEOUT 이후 침묵이 이어지면 Whisper STT 호출."""
        phrase_bytes = bytearray()
        last_audio_ts: float | None = None  # 마지막 청크 수신 시각 (monotonic)

        while rclpy.ok():
            # 1) 새 오디오 청크 대기 (최대 30 ms)
            try:
                chunk: bytes = self.data_queue.get(timeout=0.03)
                phrase_bytes.extend(chunk)
                last_audio_ts = time.monotonic()
                continue  # 청크 더 모으기
            except queue.Empty:
                pass  # 새 입력 없음 → 아래서 타임아웃 체크

            # 2) 침묵이 일정 시간 지속되면 STT 시도
            if last_audio_ts is not None and (time.monotonic() - last_audio_ts) > PHRASE_TIMEOUT and phrase_bytes:
                # PCM int16 → float32(-1~1)
                audio_np = (np.frombuffer(phrase_bytes, dtype=np.int16)
                            .astype(np.float32) / 32768.0)
                audio_np = np.clip(audio_np, -1.0, 1.0)

                # 다음 문장을 위해 리셋
                phrase_bytes.clear()
                last_audio_ts = None

                try:
                    result = self.model.transcribe(
                        audio_np,
                        device="cuda" if torch.cuda.is_available() else "cpu",
                        fp16=torch.cuda.is_available(),
                    )
                    text = result["text"].strip()
                    if text:
                        return text
                except Exception as e:
                    self.get_logger().error(f"Whisper error: {e}")
                    continue  # 오류 후 루프 지속

        return None  # 노드 종료 시

# ──────────────────  명령 노드  ──────────────────
class SpeechToCmd(Node):
    def __init__(self):
        super().__init__("speech_to_cmd")

        self.commands = self._load_commands("models/audio_cmd_list.txt")
        self.publisher_cmd = self.create_publisher(UserCommand, "/GDH_user_cmd", 1)

        # STT 클라이언트
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

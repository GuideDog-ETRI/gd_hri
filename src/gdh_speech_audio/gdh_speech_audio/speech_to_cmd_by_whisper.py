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

import threading, time

def play_audio_nonblocking(filepath: str):
    def _play():
        try:
            print('play_audio_nonblocking_start')
            pygame.mixer.init()
            print('play_audio_nonblocking_inited')
            pygame.mixer.music.load(filepath)
            print('play_audio_nonblocking_loaded')
            pygame.mixer.music.play()
            print('play_audio_nonblocking_played')
            # 최대 3초 이내에 끝날 것이므로, 여기서는 슬립으로 대체
            time.sleep(1.0)
        finally:
            pygame.mixer.quit()
            print('play_audio_nonblocking_quited')
    threading.Thread(target=_play, daemon=True).start()

import os
import yaml

# ──────────────────  Whisper STT  ──────────────────
import numpy as np
import speech_recognition as sr
import whisper, torch
import re
from queue import Queue
from datetime import datetime, timedelta
import pyaudio



class SpeechToTextClient(Node):
    """
    Whisper STT:
    listen_in_background → Queue 수신 → phrase_timeout 침묵이면 문장 완성
    외부에선 transcribe_once() 호출 시 완성된 문장(str) 또는 None 을 받는다.
    """
    def __init__(self):
        super().__init__("speech_to_text_client")
        
        # load from conf.yaml
        path_to_config = 'models/gdh_config.yaml'
        if os.path.exists(path_to_config):
            with open(path_to_config) as fid:
                conf = yaml.full_load(fid)
        else:
            raise AssertionError(f'No gdh_config file in {path_to_config}.')
        
        self.WHISPER_MODEL_NAME = conf['openai_whisper']['WHISPER_MODEL_NAME']
        self.MIC_SAMPLE_RATE   = conf['openai_whisper']['MIC_SAMPLE_RATE']
        self.ENERGY_THRESHOLD  = conf['openai_whisper']['ENERGY_THRESHOLD']
        self.RECORD_TIMEOUT    = conf['openai_whisper']['RECORD_TIMEOUT']
        self.PHRASE_TIMEOUT    = conf['openai_whisper']['PHRASE_TIMEOUT']
        
        self.get_logger().info(f"WHISPER_MODEL_NAME “{self.WHISPER_MODEL_NAME}”…")
        self.get_logger().info(f"MIC_SAMPLE_RATE “{self.MIC_SAMPLE_RATE}”…")
        self.get_logger().info(f"ENERGY_THRESHOLD “{self.ENERGY_THRESHOLD}”…")
        self.get_logger().info(f"RECORD_TIMEOUT “{self.RECORD_TIMEOUT}”…")
        self.get_logger().info(f"PHRASE_TIMEOUT “{self.PHRASE_TIMEOUT}”…")
        
        # Whisper 모델
        self.get_logger().info(f"Loading Whisper model “{self.WHISPER_MODEL_NAME}”…")
        self.model = whisper.load_model(self.WHISPER_MODEL_NAME)
        self.get_logger().info("Whisper loaded.")

        # SpeechRecognition 설정
        self.recognizer = sr.Recognizer()
        self.recognizer.energy_threshold = self.ENERGY_THRESHOLD
        self.recognizer.dynamic_energy_threshold = False

        # Original
        # self.mic = sr.Microphone(sample_rate=self.MIC_SAMPLE_RATE)
        # with self.mic:
        #     self.recognizer.adjust_for_ambient_noise(self.mic, duration=0.5)

        pa = pyaudio.PyAudio()
        input_candidates = []

        for i in range(pa.get_device_count()):
            info = pa.get_device_info_by_index(i)
            name = info.get("name", "")
            max_in = int(info.get("maxInputChannels", 0))
            max_out = int(info.get("maxOutputChannels", 0))
            rate = int(info.get("defaultSampleRate", 16000))

            self.get_logger().info(
                f"PyAudio device[{i}] name='{name}', in={max_in}, out={max_out}, rate={rate}"
            )

            if max_in > 0:
                input_candidates.append((i, name, max_in, rate))

        target = None
        pulse_device = None
        for i, name, max_in, rate in input_candidates:
            lname = name.lower()
            if "shokz" in lname or "loop110" in lname or "opencomm" in lname:
                target = (i, name, max_in, rate)
                break
            if lname == "pulse" or lname == "default":
                pulse_device = (i, name, max_in, rate)

        if target is None and pulse_device is not None:
            import subprocess
            try:
                default_src = subprocess.check_output(
                    ["pactl", "get-default-source"],
                    env={**os.environ, "PULSE_SERVER": "unix:/run/user/2002/pulse/native"},
                    timeout=3,
                ).decode().strip().lower()
                if "shokz" in default_src or "c0_86_b3_cb_a2_16" in default_src or "opencomm" in default_src:
                    target = pulse_device
                    self.get_logger().info(
                        f"Shokz is PulseAudio default source ({default_src}); "
                        f"using ALSA 'pulse' device index={pulse_device[0]}"
                    )
            except Exception as e:
                self.get_logger().warn(f"pactl get-default-source failed: {e}")

        if target is None:
            pa.terminate()
            raise RuntimeError(
                "Shokz microphone was not detected as an input device. "
                f"Input-capable devices found: {input_candidates}"
            )

        target_index, target_name, target_channels, target_rate = target

        self.get_logger().info(
            f"Using input microphone device_index={target_index}, "
            f"name='{target_name}', channels={target_channels}, rate={target_rate}"
        )

        preferred_rates = [
            int(self.MIC_SAMPLE_RATE),   # config 값
            16000,
            int(target_rate),            # PyAudio defaultSampleRate
            48000,
            44100,
            32000,
            16000,
            8000,
        ]

        actual_rate = self._find_supported_rate(pa, target_index, preferred_rates)

        self.get_logger().info(
            f"Opening microphone '{target_name}' with sample_rate={actual_rate}"
        )

        self.mic = sr.Microphone(
            device_index=target_index,
            sample_rate=actual_rate,
            chunk_size=1024,
        )

        try:
            with self.mic as source:
                if source.stream is None:
                    raise RuntimeError("Microphone stream failed to open")

                self.recognizer.adjust_for_ambient_noise(source, duration=0.5)

        except Exception as e:
            raise RuntimeError(
                f"Failed to initialize microphone '{target_name}' "
                f"(index={target_index}, sample_rate={actual_rate}, "
                f"channels={target_channels}): {e}"
            )

        # ALSA / USB audio device release 여유 시간
        self.get_logger().info("Microphone closed after calibration. Waiting before background listen...")
        time.sleep(1.0)

        pa.terminate()

        # 버퍼·큐
        self.data_queue: Queue[bytes] = Queue()
        self.phrase_bytes: bytes = b""
        self.phrase_time: datetime | None = None

        # 백그라운드 녹음 스레드
        self.recognizer.listen_in_background(
            self.mic,
            self._record_callback,
            phrase_time_limit=self.RECORD_TIMEOUT,
        )

    def _find_supported_rate(self, pa, device_index: int, preferred_rates: list[int]) -> int:
        checked = []
        for rate in preferred_rates:
            if rate in checked:
                continue
            checked.append(rate)
            try:
                pa.is_format_supported(
                    rate,
                    input_device=device_index,
                    input_channels=1,
                    input_format=pyaudio.paInt16,
                )
                self.get_logger().info(f"Supported sample_rate found: {rate}")
                return rate
            except Exception as e:
                self.get_logger().warn(f"sample_rate {rate} not supported: {e}")

        raise RuntimeError(
            f"No supported sample rate found for device_index={device_index}. "
            f"Tried: {checked}"
        )

    # ───────── 내부 콜백 ─────────
    def _record_callback(self, _sr, audio: sr.AudioData):
        self.data_queue.put(audio.get_raw_data())

    # ───────── 외부 API ─────────
    def transcribe_once_base(self) -> str | None:
        """완성된 문장이 생길 때까지 블록, 생기면 문자열·실패면 None."""
        
        # buffer initialization
        self.phrase_bytes = b""
        with self.data_queue.mutex:
            self.data_queue.queue.clear()
            
        while rclpy.ok():
            if not self.data_queue.empty():
                now = datetime.utcnow()

                # 침묵이 길었다면 새 문장
                if self.phrase_time and (now - self.phrase_time) > timedelta(seconds=self.PHRASE_TIMEOUT):
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
                    text = re.sub(r'[^가-힣]+', '', text)
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
                if self.phrase_time and (now - self.phrase_time) > timedelta(seconds=self.PHRASE_TIMEOUT):
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

        # load from conf.yaml
        path_to_config = 'models/gdh_config.yaml'
        if os.path.exists(path_to_config):
            with open(path_to_config) as fid:
                conf = yaml.full_load(fid)
        else:
            raise AssertionError(f'No gdh_config file in {path_to_config}.')
        
        self.use_trigger_word = conf['STT']['use_trigger_word']

        if self.use_trigger_word:
            self.trigger_word = conf['STT']['trigger_word']
        else:
            self.trigger_word = None
        self.get_logger().info(f"use_trigger_word: {self.use_trigger_word}")
        self.get_logger().info(f"trigger_word: {self.trigger_word}")

        self.commands = self._load_commands("models/audio_cmd_list.txt", self.trigger_word)
        self.publisher_cmd = self.create_publisher(UserCommand, "/GDH_user_cmd", 1)

        # STT
        self.stt_client = SpeechToTextClient()

        self._listen_and_publish()

    # --- 명령 리스트 로드 ---
    def _load_commands(self, path: str, trigger_word: str | None) -> dict:
        cmd = {}
        with open(path, "r") as f:
            for line in f.readlines():
                key, usr_cmd, param = line.strip().split("|")

                key_no_blank = key.replace(" ", "")
                cmd[key_no_blank] = {"usr_cmd": usr_cmd, "cmd_param": param}

                if trigger_word:
                    # 트리거 단어가 있다면 트리거 단어 + 키단어를 dict에 추가
                    key_with_trigger = f"{trigger_word}{key_no_blank}"
                    cmd[key_with_trigger] = {"usr_cmd": usr_cmd, "cmd_param": param}

        self.get_logger().info(f"Loaded commands: {list(cmd.keys())}")

        return cmd

    # --- 메인 루프 ---
    def _listen_and_publish(self):
        msg = UserCommand()
        self.get_logger().info("Start listen_and_publish")

        while rclpy.ok():
            try:
                # play_audio("models/temp/start_recording.wav")
                self.get_logger().info("Recording…")

                stt_result = self.stt_client.transcribe_once_base()
                if not stt_result:
                    self.get_logger().error("STT returned nothing.")
                    continue
                    
                self.get_logger().info(f"STT result: “{stt_result}”")
                stt_key = stt_result.replace(" ", "")
                stt_key = stt_key.replace("차자", "찾아")
                stt_key = stt_key.replace("차장", "찾아")
                stt_key = stt_key.replace("쳐자", "찾아")
                stt_key = stt_key.replace("애디", "에디")
                
                # play_audio("models/temp/stop_recording.wav")

                if stt_key in self.commands:
                    self.get_logger().info("matched and play!")
                    # play_audio("models/temp/recognized.ogg")
                    info = self.commands[stt_key]

                    # [2026-10-06] UserCommand에 없는 명령은 발행하지 않음
                    # (기본값 NONE=0 은 usr_cmd에서 GO_FORWARD=0 과 같아 전진 명령으로 오인됨)
                    if not hasattr(UserCommand, info["usr_cmd"]):
                        self.get_logger().error(f"UserCommand.{info['usr_cmd']} 없음 (gd_ifc_pkg 업데이트 필요) - 발행 안 함")
                        continue

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

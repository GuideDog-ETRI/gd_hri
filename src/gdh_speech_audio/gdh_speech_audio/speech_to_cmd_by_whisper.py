#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ------------------------------------------------------------
# ROS2 노드: SpeechToCmd  ‑‑ Whisper STT 버전
# ------------------------------------------------------------

# [2026-10-06] 간헐적 SIGSEGV(exit code -11) 원인 추적용: 크래시 순간 모든 스레드의 파이썬 스택을 stderr(launch 화면)에 출력
import faulthandler
faulthandler.enable()

# ──────────────────  ROS  ──────────────────
import rclpy
from rclpy.node import Node
from gd_ifc_pkg.msg import UserCommand
from gd_ifc_pkg.srv import GDHSpeakText
from std_msgs.msg import String

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
import json
import subprocess

# ──────────────────  Whisper STT  ──────────────────
import numpy as np
import speech_recognition as sr
import whisper, torch
import re
from queue import Queue
from datetime import datetime, timedelta
import pyaudio

# ──────────────────  [2026-10-20] 샥즈 마이크 판별  ──────────────────
# PyAudio 장치 이름, PulseAudio 소스 이름·설명 공통 (c0_86_b3_cb_a2_16 = OpenComm2 블루투스 주소)
SHOKZ_KEYWORDS = ("shokz", "opencomm", "loop110", "c0_86_b3_cb_a2_16")
PACTL_ENV = {**os.environ, "PULSE_SERVER": "unix:/run/user/2002/pulse/native", "LC_ALL": "C"}

def is_shokz(text: str) -> bool:
    text = (text or "").lower()
    return any(k in text for k in SHOKZ_KEYWORDS)

def is_shokz_source(source: dict) -> bool:
    """pactl 소스가 샥즈 마이크인지 (샥즈 출력의 .monitor 는 제외)"""
    if source["name"].endswith(".monitor"):
        return False
    return is_shokz(f"{source['name']} {source.get('description') or ''}")

def pactl_json(*args):
    out = subprocess.check_output(["pactl", "-f", "json", *args], env=PACTL_ENV, timeout=3)
    return json.loads(out)


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
            if is_shokz(name):
                target = (i, name, max_in, rate)
                break
            if lname == "pulse" or lname == "default":
                pulse_device = (i, name, max_in, rate)

        # [2026-10-20] ALSA 'pulse'/'default' 는 그때그때의 기본 입력으로 녹음하므로, 샥즈가 끊기면
        # 다른 마이크(NoMachine 가상 마이크, THETA Z1 내장 마이크 등)로 조용히 넘어감.
        # -> 샥즈가 기본 입력이 아니어도 일단 열고, _mic_guard_loop 가 녹음 소스를 샥즈로 고정·감시
        #    (샥즈로 녹음되지 않는 동안은 음성을 버리고 경고를 계속 출력)
        # 원본: 샥즈가 PulseAudio 기본 입력일 때만 사용, 아니면 RuntimeError
        self.mic_guard = target is None and pulse_device is not None
        if self.mic_guard:
            target = pulse_device
            self.get_logger().info(
                f"Using ALSA '{pulse_device[1]}' device index={pulse_device[0]} via PulseAudio; "
                f"recording source is pinned to Shokz"
            )

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

        # [2026-10-06] 보정에 쓴 마이크 스트림을 닫지 않고 백그라운드 녹음까지 그대로 사용
        # 원본: with self.mic 로 보정 후 스트림을 닫고 1초 대기 -> listen_in_background 가 스트림을 다시 엶.
        # 쉬고 있던 블루투스 헤드셋 마이크가 막 깨어난 직후에 닫았다 다시 열면 간헐적으로 SIGSEGV(exit code -11) 발생
        # [2026-10-20] 주변 소음 보정(adjust_for_ambient_noise)은 _listen_loop 에서 샥즈로 녹음될 때 수행
        try:
            source = self.mic.__enter__()
            if source.stream is None:
                raise RuntimeError("Microphone stream failed to open")

        except Exception as e:
            raise RuntimeError(
                f"Failed to initialize microphone '{target_name}' "
                f"(index={target_index}, sample_rate={actual_rate}, "
                f"channels={target_channels}): {e}"
            )

        self.get_logger().info("Microphone opened. Keeping the stream open for background listen.")

        pa.terminate()

        # 버퍼·큐
        self.data_queue: Queue[bytes] = Queue()
        self.phrase_bytes: bytes = b""
        self.phrase_time: datetime | None = None

        # [2026-10-20] 녹음 소스 감시 상태. mic_ok 가 False 인 동안 녹음된 음성은 버림
        self.mic_ok = not self.mic_guard       # 샥즈 장치를 직접 연 경우는 항상 True
        self.mic_ok_since = time.time()        # 샥즈로 녹음되기 시작한 시각 (이전에 시작된 녹음은 버림)
        self.need_calibrate = True             # 샥즈로 녹음되기 시작하면 주변 소음 보정
        if self.mic_guard:
            threading.Thread(target=self._mic_guard_loop, daemon=True).start()

        # 백그라운드 녹음 스레드 (listen_in_background 와 같은 동작, 단 이미 열린 스트림 사용)
        threading.Thread(target=self._listen_loop, daemon=True).start()

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

    # ───────── [2026-10-20] 녹음 소스 감시 ─────────
    def _mic_guard_loop(self):
        """0.5초마다 녹음 스트림이 샥즈 소스에 붙어 있는지 확인.
        다른 소스에 붙어 있으면 샥즈 소스로 옮기고, 샥즈로 녹음되지 않는 동안은 음성을 버리고 경고를 계속 출력"""
        while rclpy.ok():
            try:
                ok, detail = self._pin_to_shokz()
            except Exception as e:
                ok, detail = False, f"pactl 확인 실패: {e}"

            if not ok:
                if self.mic_ok:
                    self.mic_ok = False
                    with self.data_queue.mutex:
                        self.data_queue.queue.clear()
                self.get_logger().warning(
                    f"[MIC] 샥즈 마이크로 녹음되지 않음 - 음성 입력 무시 중: {detail}",
                    throttle_duration_sec=3.0,
                )
            elif not self.mic_ok:
                self.mic_ok_since = time.time()
                self.need_calibrate = True
                self.mic_ok = True
                self.get_logger().info(f"[MIC] 샥즈 마이크로 녹음 중: {detail}")

            time.sleep(0.5)

    def _pin_to_shokz(self) -> tuple[bool, str]:
        """이 프로세스의 녹음 스트림을 샥즈 소스로 고정. (지금 샥즈로 녹음 중인지, 설명) 반환"""
        pid = str(os.getpid())
        stream = next((o for o in pactl_json("list", "source-outputs")
                       if o.get("properties", {}).get("application.process.id") == pid), None)
        if stream is None:
            return False, "녹음 스트림을 찾지 못함"

        sources = pactl_json("list", "sources")
        current = next((s for s in sources if s["index"] == stream["source"]), None)
        if current is not None and is_shokz_source(current):
            return True, current.get("description") or current["name"]

        current_name = (current.get("description") or current["name"]) if current else f"#{stream['source']}"
        shokz = next((s for s in sources if is_shokz_source(s)), None)
        if shokz is None:
            return False, f"샥즈 입력 장치 없음 (블루투스 연결 확인), 현재 녹음 소스: {current_name}"

        # 옮긴 직후는 아직 다른 소스로 녹음된 음성이 섞여 있을 수 있으므로 다음 확인에서 정상 처리
        subprocess.check_call(["pactl", "move-source-output", str(stream["index"]), shokz["name"]],
                              env=PACTL_ENV, timeout=3)
        return False, f"녹음 소스를 '{current_name}' 에서 '{shokz.get('description') or shokz['name']}' 로 옮김"

    # ───────── 내부 콜백 ─────────
    def _listen_loop(self):
        while rclpy.ok():
            # [2026-10-20] 주변 소음 보정은 샥즈로 녹음될 때만 (시작 시, 샥즈가 다시 잡혔을 때)
            if self.need_calibrate and self.mic_ok:
                self.need_calibrate = False
                self.recognizer.energy_threshold = self.ENERGY_THRESHOLD
                self.recognizer.adjust_for_ambient_noise(self.mic, duration=0.5)
                self.get_logger().info(f"Microphone calibrated (energy_threshold={self.recognizer.energy_threshold:.0f})")

            started = time.time()
            try:  # 1초씩 듣고 다시 확인 (listen_in_background 와 동일)
                audio = self.recognizer.listen(self.mic, 1, self.RECORD_TIMEOUT)
            except sr.WaitTimeoutError:
                continue
            except Exception as e:
                self.get_logger().error(f"Background listen stopped: {e}")
                raise

            # [2026-10-20] 샥즈가 아닌 소스로 녹음됐을 수 있는 음성은 버림
            if not self.mic_ok or started < self.mic_ok_since:
                continue
            self._record_callback(self.recognizer, audio)

    def _record_callback(self, _sr, audio: sr.AudioData):
        self.data_queue.put(audio.get_raw_data())

    # ───────── 외부 API ─────────
    def transcribe_once_base(self, timeout_sec: float | None = None) -> str | None:
        """완성된 문장이 생길 때까지 블록, 생기면 문자열·실패면 None.
        [2026-10-06] 반환값은 Whisper 원문 (대화 모드에서 VLM 질문으로 그대로 사용, 명령 매칭 키는 호출하는 쪽에서 만듦).
        timeout_sec 동안 문장이 없으면 None (대화 모드 자동 종료용, None 이면 무제한 대기)."""
        deadline = None if timeout_sec is None else time.time() + timeout_sec

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
                    # [2026-10-06] 원본: text = re.sub(r'[^가-힣]+', '', text) 를 반환 (공백·숫자·문장부호 제거)
                    if re.sub(r'[^가-힣]+', '', text):
                        return text
                except Exception as e:
                    self.get_logger().error(f"Whisper error: {e}")
                    return None
            else:
                if deadline is not None and time.time() >= deadline:
                    return None
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

        # [2026-10-06] 대화 모드: start_words ~ end_words 사이에서 명령에 매칭되지 않는 문장은
        # 원문 그대로 /GDH_dialog_text 로 보내 VLM 자유 질문으로 처리 (image_ai_node)
        dialog_conf = conf['STT'].get('dialog') or {}
        self.dialog_start_words = self._load_words(dialog_conf.get('start_words', []), self.trigger_word)
        self.dialog_end_words   = self._load_words(dialog_conf.get('end_words', []), self.trigger_word)
        self.dialog_use_timeout = bool(dialog_conf.get('use_timeout', False))
        self.dialog_timeout_sec = float(dialog_conf.get('timeout_sec', 10.0))
        self.dialog_start_sentence = dialog_conf.get('start_sentence', '')
        self.dialog_end_sentence   = dialog_conf.get('end_sentence', '')
        self.dialog_mode = False
        self.dialog_last_time = 0.0
        self.get_logger().info(f"dialog start_words: {sorted(self.dialog_start_words)}")
        self.get_logger().info(f"dialog end_words: {sorted(self.dialog_end_words)}")
        self.get_logger().info(f"dialog use_timeout: {self.dialog_use_timeout}, timeout_sec: {self.dialog_timeout_sec}")
        self.publisher_dialog = self.create_publisher(String, "/GDH_dialog_text", 1)
        self.speak_client = self.create_client(GDHSpeakText, '/GDH_speak_text')

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

    # --- [2026-10-06] 대화 모드 ---
    def _load_words(self, words: list, trigger_word: str | None) -> set:
        """명령 키와 같은 방식: 공백 제거, 트리거 단어를 붙인 것도 인식"""
        keys = set()
        for w in words:
            w = w.replace(" ", "")
            keys.add(w)
            if trigger_word:
                keys.add(f"{trigger_word}{w}")
        return keys

    def _dialog_time_left(self) -> float | None:
        """대화 모드 자동 종료까지 남은 시간(초). 대화 모드가 아니거나 자동 종료를 쓰지 않으면 None"""
        if not (self.dialog_mode and self.dialog_use_timeout):
            return None
        return max(0.0, self.dialog_timeout_sec - (time.time() - self.dialog_last_time))

    def _set_dialog_mode(self, on: bool, reason: str):
        self.dialog_mode = on
        self.dialog_last_time = time.time()
        self.get_logger().info(f"Dialog mode {'ON' if on else 'OFF'} ({reason})")
        self._speak(self.dialog_start_sentence if on else self.dialog_end_sentence)

    def _speak(self, text: str):
        """안내 음성 (msg_to_audio 의 /GDH_speak_text). 응답은 기다리지 않음"""
        if not text:
            return
        if not self.speak_client.service_is_ready():
            self.get_logger().warning(f"/GDH_speak_text 서비스 없음 - 안내 생략: {text}")
            return
        req = GDHSpeakText.Request()
        req.text = text
        self.speak_client.call_async(req)

    # --- 메인 루프 ---
    def _listen_and_publish(self):
        msg = UserCommand()
        self.get_logger().info("Start listen_and_publish")

        while rclpy.ok():
            try:
                # play_audio("models/temp/start_recording.wav")
                self.get_logger().info("Recording…")

                stt_result = self.stt_client.transcribe_once_base(timeout_sec=self._dialog_time_left())
                if not stt_result:
                    # [2026-10-06] 대화 모드에서 timeout_sec 동안 인식된 문장이 없으면 자동 종료
                    if self.dialog_mode and self._dialog_time_left() == 0:
                        self._set_dialog_mode(False, f"{self.dialog_timeout_sec:.0f}초 동안 말이 없음")
                    else:
                        self.get_logger().error("STT returned nothing.")
                    continue

                self.get_logger().info(f"STT result: “{stt_result}”")
                if self.dialog_mode:
                    self.dialog_last_time = time.time()

                # 명령 매칭 키: 한글만 남김 (공백·숫자·문장부호 제거)
                stt_key = re.sub(r'[^가-힣]+', '', stt_result)
                stt_key = stt_key.replace("차자", "찾아")
                stt_key = stt_key.replace("차장", "찾아")
                stt_key = stt_key.replace("쳐자", "찾아")
                stt_key = stt_key.replace("애디", "에디")
                
                # play_audio("models/temp/stop_recording.wav")

                # [2026-10-06] 대화 모드 시작/종료
                if stt_key in self.dialog_start_words:
                    self._set_dialog_mode(True, stt_key)
                    continue
                if self.dialog_mode and stt_key in self.dialog_end_words:
                    self._set_dialog_mode(False, stt_key)
                    continue

                if stt_key in self.commands:
                    self.get_logger().info("matched and play!")
                    # 인식 알림음: recognized.ogg 대신 TTS 서버에 효과음 재생 요청
                    self._speak("@play notify")
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
                elif self.dialog_mode:
                    # [2026-10-06] 대화 모드: 명령이 아닌 문장은 원문 그대로 VLM 자유 질문으로
                    self.publisher_dialog.publish(String(data=stt_result))
                    self.get_logger().info(f"Dialog question published: {stt_result}")
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

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
# [2026-10-23] 오디오 서버가 바쁠 때 pactl 응답이 3~5초 걸림 (NoMachine 접속 시 WirePlumber 2개가 사운드카드 경쟁)
# 기존 timeout=3 -> 수시로 timeout 나서 샥즈로 녹음 중인데도 음성을 버림
PACTL_TIMEOUT = 10

def is_shokz(text: str) -> bool:
    text = (text or "").lower()
    return any(k in text for k in SHOKZ_KEYWORDS)

def is_shokz_source(source: dict) -> bool:
    """pactl 소스가 샥즈 마이크인지 (샥즈 출력의 .monitor 는 제외)"""
    if source["name"].endswith(".monitor"):
        return False
    return is_shokz(f"{source['name']} {source.get('description') or ''}")

def pactl_json(*args):
    # [2026-10-23] 원본 (timeout=3)
    # out = subprocess.check_output(["pactl", "-f", "json", *args], env=PACTL_ENV, timeout=3)
    out = subprocess.check_output(["pactl", "-f", "json", *args], env=PACTL_ENV, timeout=PACTL_TIMEOUT)
    return json.loads(out)

# ──────────────────  [2026-10-30] 음성 인식 결과 콘솔 출력  ──────────────────
# 받아쓴 문장마다 처리 결과를 gdh 실행 창에 한 줄로 눈에 띄게 출력 (로그 줄 사이에서 찾기 쉽게 색 구분).
# gdh_config.yaml 의 STT.console_print: False 로 끔
CONSOLE_PRINT = True
from datetime import timezone
_KST = timezone(timedelta(hours=9), "KST")   # gdh 컨테이너는 시간대가 UTC 라서 한국 시각으로 표시
_CONSOLE_COLORS = {"명령": "\033[1;32m", "대화": "\033[1;36m", "정지": "\033[1;35m",
                   "매칭 없음": "\033[33m", "버림": "\033[90m"}

def console_stt(text: str, result: str, kind: str):
    """kind: 명령 / 대화 / 정지 / 매칭 없음 / 버림 (색 구분)"""
    if not CONSOLE_PRINT:
        return
    if len(text) > 60:
        text = text[:60] + "…"
    print(f"{_CONSOLE_COLORS.get(kind, '')}🎤 [{datetime.now(_KST):%H:%M:%S}] “{text}” → {result}\033[0m", flush=True)


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
        # [2026-10-23] Whisper 인식 언어·힌트 문장·환각 필터, 주변 소음 보정 상한 (없으면 기본값)
        self.WHISPER_LANGUAGE  = conf['openai_whisper'].get('LANGUAGE', 'ko')
        self.INITIAL_PROMPT    = conf['openai_whisper'].get('INITIAL_PROMPT') or None
        self.MIN_AVG_LOGPROB   = float(conf['openai_whisper'].get('MIN_AVG_LOGPROB', -1.0))
        # self.MAX_ENERGY_THRESHOLD = float(conf['openai_whisper'].get('MAX_ENERGY_THRESHOLD', 1500))  # [2026-10-23] 원본 기본값
        self.MAX_ENERGY_THRESHOLD = float(conf['openai_whisper'].get('MAX_ENERGY_THRESHOLD', 1000))
        # [2026-10-23] Whisper 재시도(temperature fallback) 끔: 잡음을 받아쓸 때 최대 7.5초 멈춰 다음 명령이 늦거나 무시됨
        self.WHISPER_TEMPERATURE = float(conf['openai_whisper'].get('TEMPERATURE', 0.0))
        # [2026-10-23] 받아쓰기 대기열에서 이보다 오래된 말만 버림 (원본: 받아쓰기 시작할 때마다 모두 버림)
        self.MAX_QUEUE_AGE_SEC   = float(conf['openai_whisper'].get('MAX_QUEUE_AGE_SEC', 5.0))
        # [2026-10-23] 재시도를 끈 뒤 가끔 같은 말이 반복되는 결과("어, 어, 어, ...") -> 반복이 심하면 버림
        self.MAX_COMPRESSION_RATIO = float(conf['openai_whisper'].get('MAX_COMPRESSION_RATIO', 2.0))
        # [2026-10-23] 마이크(샥즈)가 끊기거나 다시 연결되면 음성 안내 (로그만으로는 사용자가 알 수 없음)
        alert_conf = (conf.get('STT') or {}).get('mic_alert') or {}
        self.MIC_ALERT_ENABLE   = bool(alert_conf.get('enable', True))
        self.MIC_ALERT_LOST_SEC = float(alert_conf.get('lost_after_sec', 5.0))
        self.MIC_ALERT_LOST_SENTENCE     = alert_conf.get('lost_sentence', "마이크 연결이 끊겼습니다. 음성 명령을 들을 수 없습니다.")
        self.MIC_ALERT_RESTORED_SENTENCE = alert_conf.get('restored_sentence', "마이크가 다시 연결되었습니다.")
        self.speak_client = self.create_client(GDHSpeakText, '/GDH_speak_text')
        # [2026-10-23] STT 가 살아 있음을 1초마다 알림. 끊기면 TTS 노드(msg_to_audio)가 "음성 인식 중단" 안내
        self.alive_pub = self.create_publisher(String, '/GDH_stt_alive', 10)

        self.get_logger().info(f"WHISPER_MODEL_NAME “{self.WHISPER_MODEL_NAME}”…")
        self.get_logger().info(f"MIC_SAMPLE_RATE “{self.MIC_SAMPLE_RATE}”…")
        self.get_logger().info(f"ENERGY_THRESHOLD “{self.ENERGY_THRESHOLD}”…")
        self.get_logger().info(f"RECORD_TIMEOUT “{self.RECORD_TIMEOUT}”…")
        self.get_logger().info(f"PHRASE_TIMEOUT “{self.PHRASE_TIMEOUT}”…")
        self.get_logger().info(f"LANGUAGE “{self.WHISPER_LANGUAGE}”, INITIAL_PROMPT “{self.INITIAL_PROMPT}”, "
                               f"MIN_AVG_LOGPROB {self.MIN_AVG_LOGPROB}, MAX_ENERGY_THRESHOLD {self.MAX_ENERGY_THRESHOLD:.0f}, "
                               f"TEMPERATURE {self.WHISPER_TEMPERATURE}, MAX_QUEUE_AGE_SEC {self.MAX_QUEUE_AGE_SEC}")
        
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
        # [2026-10-23] 원본: self.data_queue: Queue[bytes] = Queue()
        # 말 하나 = (PCM, RECORD_TIMEOUT 에서 잘렸는지, 대기열에 넣은 시각)
        self.data_queue: Queue[tuple[bytes, bool, float]] = Queue()
        self.prev_cut = False                  # [2026-10-23] 앞 말이 잘렸으면 다음 말을 이어 붙임
        self.phrase_bytes: bytes = b""
        self.phrase_time: datetime | None = None

        # [2026-10-20] 녹음 소스 감시 상태. mic_ok 가 False 인 동안 녹음된 음성은 버림
        self.mic_ok = not self.mic_guard       # 샥즈 장치를 직접 연 경우는 항상 True
        self.mic_ok_since = time.time()        # 샥즈로 녹음되기 시작한 시각 (이전에 시작된 녹음은 버림)
        self.need_calibrate = True             # 샥즈로 녹음되기 시작하면 주변 소음 보정
        # [2026-10-23] 마이크 끊김 안내 상태: 샥즈로 녹음되지 않기 시작한 시각, 끊김 안내를 했는지
        self.mic_lost_since = time.time()
        self.mic_lost_announced = False
        if self.mic_guard:
            threading.Thread(target=self._mic_guard_loop, daemon=True).start()

        # 백그라운드 녹음 스레드 (listen_in_background 와 같은 동작, 단 이미 열린 스트림 사용)
        # [2026-10-23] 원본: threading.Thread(target=self._listen_loop, daemon=True).start()
        threading.Thread(target=self._listen_loop_or_exit, daemon=True).start()
        # [2026-10-23] 녹음을 시작한 뒤부터 살아 있음 신호 (/GDH_stt_alive)
        threading.Thread(target=self._alive_loop, daemon=True).start()

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
                # [2026-10-23] pactl 실패(응답 지연)는 샥즈가 아니라는 뜻이 아님 -> 직전 상태 유지
                # 원본 (샥즈로 녹음 중인데도 음성을 버리고 다시 보정 -> 실측: 시간의 약 2/3 동안 음성 무시)
                # ok, detail = False, f"pactl 확인 실패: {e}"
                self.get_logger().warning(
                    f"[MIC] pactl 확인 실패 - 직전 상태 유지 ({'샥즈 녹음 중' if self.mic_ok else '음성 무시 중'}): {e}",
                    throttle_duration_sec=10.0,
                )
                time.sleep(0.5)
                continue

            if not ok:
                if self.mic_ok:
                    self.mic_ok = False
                    self.mic_lost_since = time.time()   # [2026-10-23] 끊김 안내용
                    with self.data_queue.mutex:
                        self.data_queue.queue.clear()
                self.get_logger().warning(
                    f"[MIC] 샥즈 마이크로 녹음되지 않음 - 음성 입력 무시 중: {detail}",
                    throttle_duration_sec=3.0,
                )
                # [2026-10-23] lost_after_sec 이상 샥즈로 녹음되지 않으면 한 번 음성 안내
                if (self.MIC_ALERT_ENABLE and not self.mic_lost_announced
                        and time.time() - self.mic_lost_since >= self.MIC_ALERT_LOST_SEC):
                    self.mic_lost_announced = self._speak(self.MIC_ALERT_LOST_SENTENCE)
            elif not self.mic_ok:
                self.mic_ok_since = time.time()
                self.need_calibrate = True
                self.mic_ok = True
                self.get_logger().info(f"[MIC] 샥즈 마이크로 녹음 중: {detail}")
                # [2026-10-23] 끊김 안내를 했던 경우에만 복구 안내
                if self.mic_lost_announced:
                    self._speak(self.MIC_ALERT_RESTORED_SENTENCE)
                    self.mic_lost_announced = False

            time.sleep(0.5)

    def _speak(self, text: str) -> bool:
        """[2026-10-23] 마이크 상태 안내 음성 (msg_to_audio 의 /GDH_speak_text). 응답은 기다리지 않음.
        서비스가 아직 없으면 False (다음 확인 때 다시 시도)"""
        if not text:
            return True
        if not self.speak_client.service_is_ready():
            self.get_logger().warning(f"[MIC] /GDH_speak_text 서비스 없음 - 안내 대기: {text}", throttle_duration_sec=10.0)
            return False
        req = GDHSpeakText.Request()
        req.text = text
        self.speak_client.call_async(req)
        self.get_logger().info(f"[MIC] 안내: {text}")
        return True

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
        # [2026-10-23] 원본 (timeout=3)
        # subprocess.check_call(["pactl", "move-source-output", str(stream["index"]), shokz["name"]],
        #                       env=PACTL_ENV, timeout=3)
        subprocess.check_call(["pactl", "move-source-output", str(stream["index"]), shokz["name"]],
                              env=PACTL_ENV, timeout=PACTL_TIMEOUT)
        return False, f"녹음 소스를 '{current_name}' 에서 '{shokz.get('description') or shokz['name']}' 로 옮김"

    # ───────── 내부 콜백 ─────────
    def _alive_loop(self):
        """[2026-10-23] 1초마다 /GDH_stt_alive 발행 ("listening" 또는 샥즈로 녹음되지 않을 때 "mic_lost").
        STT 프로세스가 죽거나 녹음 스레드 오류로 종료되면 끊기므로, TTS 노드가 이를 보고 음성 안내"""
        while rclpy.ok():
            try:
                self.alive_pub.publish(String(data="listening" if self.mic_ok else "mic_lost"))
            except Exception as e:
                self.get_logger().warning(f"[STT] /GDH_stt_alive 발행 실패: {e}", throttle_duration_sec=10.0)
            time.sleep(1.0)

    def _listen_loop_or_exit(self):
        """[2026-10-23] 녹음 스레드가 오류로 멈추면 프로세스는 살아 있는데 아무것도 못 듣는 상태가 됨
        (예: 오디오 서버 재시작으로 녹음 스트림이 끊김) -> 프로세스를 종료해 launch 의 respawn 으로 STT 를 다시 띄움"""
        try:
            self._listen_loop()
        except Exception as e:
            self.get_logger().fatal(f"[STT] 녹음 스레드 오류 - STT 종료 (launch respawn 으로 다시 시작): {e}")
            os._exit(1)

    def _listen_loop(self):
        while rclpy.ok():
            # [2026-10-20] 주변 소음 보정은 샥즈로 녹음될 때만 (시작 시, 샥즈가 다시 잡혔을 때)
            if self.need_calibrate and self.mic_ok:
                self.need_calibrate = False
                self.recognizer.energy_threshold = self.ENERGY_THRESHOLD
                self.recognizer.adjust_for_ambient_noise(self.mic, duration=0.5)
                # [2026-10-23] 보정 중에 말소리가 섞이면 기준이 2000~3500 으로 튀어 평소 말소리를 놓침 -> 상한 적용
                if self.recognizer.energy_threshold > self.MAX_ENERGY_THRESHOLD:
                    self.get_logger().warning(
                        f"Microphone calibration too high ({self.recognizer.energy_threshold:.0f}, 보정 중 말소리 섞임) "
                        f"-> {self.MAX_ENERGY_THRESHOLD:.0f}")
                    self.recognizer.energy_threshold = self.MAX_ENERGY_THRESHOLD
                self.get_logger().info(f"Microphone calibrated (energy_threshold={self.recognizer.energy_threshold:.0f})")

            started = time.time()
            try:  # 1초씩 듣고 다시 확인 (listen_in_background 와 동일)
                audio = self.recognizer.listen(self.mic, 1, self.RECORD_TIMEOUT)
            except sr.WaitTimeoutError:
                continue
            except Exception as e:
                self.get_logger().error(f"Background listen stopped: {e}")
                raise

            # [2026-10-23] 진단 로그: 말소리 하나마다 길이·크기·처리 결과
            raw = audio.get_raw_data()
            pcm = np.frombuffer(raw, dtype=np.int16).astype(np.float32)
            dur = len(pcm) / audio.sample_rate
            rms = float(np.sqrt(np.mean(pcm ** 2))) if len(pcm) else 0.0
            tail = pcm[-int(0.3 * audio.sample_rate):]
            # 정상적으로 끝난 말은 끝에 침묵이 붙음. 끝까지 소리가 크면 RECORD_TIMEOUT 에서 잘린 것
            cut = len(tail) > 0 and float(np.sqrt(np.mean(tail ** 2))) > self.recognizer.energy_threshold

            # [2026-10-20] 샥즈가 아닌 소스로 녹음됐을 수 있는 음성은 버림
            if not self.mic_ok or started < self.mic_ok_since:
                self.get_logger().info(f"[VOICE] {dur:.1f}s rms={rms:.0f} -> 버림 (샥즈 녹음 상태 아님)")
                continue
            self.get_logger().info(
                f"[VOICE] {dur:.1f}s rms={rms:.0f} (기준 {self.recognizer.energy_threshold:.0f})"
                f"{' - RECORD_TIMEOUT 에서 잘림' if cut else ''} -> Whisper")
            # [2026-10-23] 원본: self._record_callback(self.recognizer, audio)
            self._record_callback(self.recognizer, audio, cut)

    # [2026-10-23] 원본
    # def _record_callback(self, _sr, audio: sr.AudioData):
    #     self.data_queue.put(audio.get_raw_data())
    def _record_callback(self, _sr, audio: sr.AudioData, cut: bool = False):
        self.data_queue.put((audio.get_raw_data(), cut, time.time()))

    # ───────── 외부 API ─────────
    def transcribe_once_base(self, timeout_sec: float | None = None) -> str | None:
        """완성된 문장이 생길 때까지 블록, 생기면 문자열·실패면 None.
        [2026-10-06] 반환값은 Whisper 원문 (대화 모드에서 VLM 질문으로 그대로 사용, 명령 매칭 키는 호출하는 쪽에서 만듦).
        timeout_sec 동안 문장이 없으면 None (대화 모드 자동 종료용, None 이면 무제한 대기)."""
        deadline = None if timeout_sec is None else time.time() + timeout_sec

        # buffer initialization
        self.phrase_bytes = b""
        self.prev_cut = False
        # [2026-10-23] 원본 (받아쓰기를 시작할 때마다 대기열을 모두 비움 -> 앞 문장을 처리하는 동안 한 말이 버려질 수 있음)
        # with self.data_queue.mutex:
        #     self.data_queue.queue.clear()
        # MAX_QUEUE_AGE_SEC 보다 오래된 말만 버림
        with self.data_queue.mutex:
            now_t = time.time()
            fresh = [item for item in self.data_queue.queue if now_t - item[2] <= self.MAX_QUEUE_AGE_SEC]
            dropped = len(self.data_queue.queue) - len(fresh)
            self.data_queue.queue.clear()
            self.data_queue.queue.extend(fresh)
        if dropped:
            self.get_logger().info(f"[VOICE] {self.MAX_QUEUE_AGE_SEC:.0f}초 넘게 지난 말 {dropped}개 버림")

        while rclpy.ok():
            if not self.data_queue.empty():
                # [2026-10-23] 원본 (대기열에 쌓인 말을 모두 이어 붙여 한 번에 받아씀)
                #   -> Whisper 가 느린 동안 쌓인 서로 다른 말이 합쳐져 명령 매칭 실패
                #      (실측: "지하철 입구 찾아. 화이트 노이즈를 좀 섞어야 될 것 같은데.")
                # now = datetime.utcnow()
                #
                # # 침묵이 길었다면 새 문장
                # if self.phrase_time and (now - self.phrase_time) > timedelta(seconds=self.PHRASE_TIMEOUT):
                #     self.phrase_bytes = b""
                #
                # self.phrase_time = now
                #
                # # 큐 비우기
                # while not self.data_queue.empty():
                #     self.phrase_bytes += self.data_queue.get()

                # 말 하나씩 받아씀. 앞 말이 RECORD_TIMEOUT 에서 잘린 경우에만 이어 붙임 (같은 문장의 뒷부분)
                data, cut, _ = self.data_queue.get()
                if not self.prev_cut:
                    self.phrase_bytes = b""
                self.phrase_bytes += data
                self.prev_cut = cut

                # PCM int16 → float32(-1~1)
                audio_np = (np.frombuffer(self.phrase_bytes, dtype=np.int16)
                            .astype(np.float32) / 32768.0)
                try:
                    # [2026-10-23] 언어 고정 + 호출어/명령 힌트 문장 ("에디야"가 "애리야"로 들리던 문제)
                    # 원본 (언어 자동 판별, 힌트 없음)
                    # result = self.model.transcribe(
                    #     audio_np,
                    #     fp16=torch.cuda.is_available(),
                    # )
                    started = time.time()
                    result = self.model.transcribe(
                        audio_np,
                        fp16=torch.cuda.is_available(),
                        language=self.WHISPER_LANGUAGE,
                        initial_prompt=self.INITIAL_PROMPT,
                        # [2026-10-23] 원본: temperature 지정 없음 (기본 0.0~1.0 최대 6번 재시도 -> 잡음에서 최대 7.5초)
                        temperature=self.WHISPER_TEMPERATURE,
                    )
                    text = result["text"].strip()
                    logprob = min((s["avg_logprob"] for s in result.get("segments", [])), default=None)
                    self.get_logger().info(
                        f"[WHISPER] {len(audio_np) / self.mic.SAMPLE_RATE:.1f}s -> “{text}” "
                        f"(avg_logprob={logprob if logprob is None else round(logprob, 2)}, {time.time() - started:.2f}s)")
                    # [2026-10-23] 무음·잡음에서 지어낸 문장("감사합니다", "다음 영상에서 만나요") 버림
                    # 실측: 실제 말 -0.04~-0.27, 지어낸 문장 -2.6~-7.9 (힌트 문장 사용 시)
                    if logprob is not None and logprob < self.MIN_AVG_LOGPROB:
                        self.get_logger().info(f"[WHISPER] 신뢰도 낮음 (< {self.MIN_AVG_LOGPROB}) - 버림")
                        console_stt(text, f"버림 (신뢰도 낮음 {logprob:.2f})", "버림")   # [2026-10-30]
                        continue
                    # [2026-10-23] 같은 말이 반복되는 결과 버림 (Whisper compression_ratio: 반복이 많을수록 큼)
                    # 실측(오늘 667건): 실제 명령 문장 최대 1.35, 반복 문장 2.35("전통, 전통, ...")·27.9("어, 어, ...")
                    ratio = max((s.get("compression_ratio", 0.0) for s in result.get("segments", [])), default=0.0)
                    if ratio > self.MAX_COMPRESSION_RATIO:
                        self.get_logger().info(f"[WHISPER] 반복 문장 (compression_ratio {ratio:.2f} > {self.MAX_COMPRESSION_RATIO}) - 버림")
                        console_stt(text, "버림 (반복 문장)", "버림")   # [2026-10-30]
                        continue
                    # [2026-10-06] 원본: text = re.sub(r'[^가-힣]+', '', text) 를 반환 (공백·숫자·문장부호 제거)
                    if re.sub(r'[^가-힣]+', '', text):
                        return text
                    elif text:   # [2026-10-30] 한글이 없는 결과도 콘솔에 표시
                        console_stt(text, "버림 (한글 없음)", "버림")
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
                    # [2026-10-23] 원본: self.phrase_bytes += self.data_queue.get()  (대기열 항목이 (PCM, 잘림, 시각) 으로 바뀜)
                    self.phrase_bytes += self.data_queue.get()[0]
    
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

        # [2026-10-30] "조용히 해": 인식 문장에 정지 단어가 들어 있으면 TTS 서버에 code("@quiet")를 보내
        # 쌓인 말을 버리고 말하던 문장을 끊고, /GDH_quiet 를 발행해 VLM 이 만들던 답도 멈춤 (image_ai_node -> VLM).
        # 설계: docs/stop_speech_design_261029.md. 설정이 없으면 꺼짐
        stop_conf = conf['STT'].get('stop_speech') or {}
        self.stop_enable = bool(stop_conf.get('enable', False))
        self.stop_words = [k for k in (str(w).replace(" ", "") for w in stop_conf.get('words') or []) if k]
        self.stop_code = stop_conf.get('code') or "@quiet"
        self.get_logger().info(f"stop_speech enable: {self.stop_enable}, words: {self.stop_words}, code: {self.stop_code}")
        self.publisher_quiet = self.create_publisher(String, "/GDH_quiet", 1)

        # [2026-10-30] 음성 인식 결과 콘솔 출력 (console_stt)
        global CONSOLE_PRINT
        CONSOLE_PRINT = bool(conf['STT'].get('console_print', True))
        self.get_logger().info(f"console_print: {CONSOLE_PRINT}")

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
                # [2026-10-23] 실측: "에디야"가 "애리야"/"에리아"로 들림
                for heard in ("애리야", "에리야", "애리아", "에리아"):
                    stt_key = stt_key.replace(heard, "에디야")
                # [2026-10-23] 실측: "지금 상황 어때"가 "지금의 상황 어때"로 들림
                stt_key = stt_key.replace("지금의", "지금")
                
                # play_audio("models/temp/stop_recording.wav")

                # [2026-10-30] "조용히 해": 대화 모드 시작/종료·명령 매칭보다 먼저 검사
                #   (늦게 검사하면 대화 모드에서 VLM 질문으로 넘어가 또 답을 말함). 기존 명령과 달리 "들어 있으면" 정지
                #   1) TTS 서버: 쌓인 말 버리고 말하던 문장 끊기  2) /GDH_quiet -> image_ai_node -> VLM 생성 중단
                if self.stop_enable and any(w in stt_key for w in self.stop_words):
                    self._speak(self.stop_code)
                    # 다른 명령과 같이 인식 알림음. "@quiet" 다음에 보냄 (말을 먼저 끊음, 효과음은 별도 재생기라 @quiet 에 안 끊김)
                    self._speak("@play notify")
                    self.publisher_quiet.publish(String(data=stt_result))
                    self.get_logger().info(f"[STOP] “{stt_result}” -> {self.stop_code}, /GDH_quiet")
                    console_stt(stt_result, f"정지 ({self.stop_code})", "정지")
                    continue

                # [2026-10-06] 대화 모드 시작/종료
                if stt_key in self.dialog_start_words:
                    self._set_dialog_mode(True, stt_key)
                    console_stt(stt_result, "대화 모드 시작", "대화")   # [2026-10-30]
                    continue
                if self.dialog_mode and stt_key in self.dialog_end_words:
                    self._set_dialog_mode(False, stt_key)
                    console_stt(stt_result, "대화 모드 종료", "대화")   # [2026-10-30]
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
                        console_stt(stt_result, f"명령 {info['usr_cmd']} 없음 (gd_ifc_pkg) - 발행 안 함", "매칭 없음")   # [2026-10-30]
                        continue

                    msg.usr_cmd   = getattr(UserCommand, info["usr_cmd"],   UserCommand.NONE)
                    msg.cmd_param = getattr(UserCommand, info["cmd_param"], UserCommand.NONE)

                    self.publisher_cmd.publish(msg)
                    self.get_logger().info(f"Published: {msg.usr_cmd}, {msg.cmd_param}")
                    console_stt(stt_result, f"명령 {info['usr_cmd']}", "명령")   # [2026-10-30]
                elif self.dialog_mode:
                    # [2026-10-06] 대화 모드: 명령이 아닌 문장은 원문 그대로 VLM 자유 질문으로
                    self.publisher_dialog.publish(String(data=stt_result))
                    self.get_logger().info(f"Dialog question published: {stt_result}")
                    console_stt(stt_result, "VLM 질문 (대화 모드)", "대화")   # [2026-10-30]
                else:
                    self.get_logger().info("No matched command.")
                    console_stt(stt_result, "매칭 없음", "매칭 없음")   # [2026-10-30]
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

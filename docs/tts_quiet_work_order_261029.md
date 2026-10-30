# TTS 서버 작업 지시서 — `@quiet` (발화 즉시 정지)

- 작성: 2026-10-29 / 수정: 2026-10-30 (u-pc Claude 검토 의견 반영, "세대 번호만" 방식으로 확정, 원본 보존·되돌리기 절차 추가)
- 작성자: Thor(192.168.2.20) gdh 음성 인식 담당 Claude
- 대상: u-pc(192.168.2.12) `/home/u-pc/TTS_Monitoring/tts-kor-server.py` (2026-10-26 16:32 버전, 483줄, md5 `11d7bb7d82a000369ffdde2c78772df1` — 2026-10-30 09:40 확인)
- 관련 설계서(Thor): `gd_hri/docs/stop_speech_design_261029.md`
- 검토 기록: u-pc Claude 세션 `ba260f25-e223-481c-b347-0968f0668df7` (2026-10-29)

---

## 1. 배경

- 사용자가 "조용히 해"라고 하면 Thor gdh 음성 인식이 4070 포트로 `@quiet` 를 보낸다.
- 서버는 받는 즉시 쌓여 있는 말을 모두 버리고, 말하던 문장도 바로 끊어야 한다.
- 발화 큐는 이 서버에만 있다. gdh·VLM3·gdm·gds 가 모두 4070 으로 직접 보내기 때문에, 이 작업은 서버에서만 할 수 있다.

## 2. 받는 메시지

기존 포트·형식 그대로: 4070, `float32 속도(4바이트)` + `UTF-8 텍스트`.

| 텍스트 (`strip()` 후) | 동작 |
|---|---|
| `@quiet` | 쌓인 말을 모두 버리고 말하던 문장을 끊는다. 속도 값은 무시. 인자 없음 |

- 보내는 쪽
  - Thor gdh `msg_to_audio` (사용자가 "조용히 해"라고 했을 때)
  - Thor VLM3 `guidedog-cli` (정지 요청을 받아 생성을 멈출 때 한 번, 그 직전에 보낸 조각 정리용)
  - 같은 `@quiet` 가 수십 ms 간격으로 두 번 올 수 있다.
- 기존 명령은 그대로 둔다: `@play 이름`(음악·효과음), `@stop`(**음악 정지**).
  - `@quiet` 는 음악·효과음(`music_player`)을 건드리지 않는다.
- **정지 뒤에 도착한 메시지는 평소처럼 말한다** (무시 구간 없음).

## 3. 방식: 세대 번호 (2026-10-30 사용자 결정)

- 큐에 넣는 모든 메시지에 그때의 세대 번호 `_gen` 을 붙인다.
- `@quiet` 를 받으면 수신 루프가 `_gen` 을 1 올린다. 그 외에는 아무것도 하지 않는다.
- 합성·재생 워커는 항목을 꺼낼 때, 그리고 조각·블록마다 번호를 확인한다. 옛 세대면 말하지 않고 버린다.
- 큐를 따로 뒤지는 정지 전용 쓰레드는 만들지 않는다.
  - 수신 루프 자체가 합성·재생 쓰레드와 따로 돌기 때문이다.
  - 옛 항목은 워커가 꺼내는 순간 건너뛰므로, 소리가 멈추는 시간은 같다.
- 정지 뒤에 들어온 메시지는 새 세대이므로 자연히 보존된다.

## 4. 요구사항

| # | 요구 |
|---|---|
| R1 | **수신 루프** (432~476행): `@quiet` 는 `@play`/`@stop` 을 판별하는 곳(451~461행, sanitize·영문 변환 전)에서 같이 판별한다. 큐에 넣지 않고 `_gen += 1` 한 뒤 로그를 남기고 바로 다음 메시지를 받는다 |
| R2 | **큐에 넣을 때**: `(text, speed, _gen)` 으로 넣는다. `@play` 파일이 없을 때 넣는 안내 문장(456행)도 같은 함수(`enqueue_tts`)를 쓴다 |
| R3 | **tts_worker**: 꺼냈을 때 옛 세대면 합성하지 않는다. 문장 조각을 하나씩 받으므로(`text_to_speech` 가 yield) 조각마다 확인하고, 옛 세대가 되면 남은 조각은 합성하지 않는다. `audio_queue` 에는 `(audio, gen)` 으로 넣는다 |
| R4 | **audio_worker**: 꺼냈을 때 옛 세대면 재생하지 않는다. 재생은 `sd.play` + `sd.wait` 대신 `sd.OutputStream(..., blocksize=1024, latency='low')` 에 **1024 샘플(≈21ms) 블록** 단위로 쓰고, 블록마다 세대를 확인한다. 옛 세대면 **10~20ms(480~960 샘플)** 페이드아웃을 쓰고 멈춘다. 쓰는 데이터는 모두 `float32` 로 맞춘다 (페이드 줄 포함) |
| R5 | **다른 쓰레드에서 `sd.stop()` 을 부르지 않는다.** 10-26 첫 시험에서 `sd.wait()` 와 교착되어 서버가 멈췄다. `with` 블록이 끝날 때 `stop()` 이 불려 이미 버퍼에 들어간 페이드아웃까지 재생되는 것은 정상이다 (`abort()` 는 "딱" 소리가 나므로 쓰지 않음) |
| R6 | **underflow 로그**: `st.write()` 가 underflow 를 알려 주면 로그를 남긴다. 합성 중 GIL 때문에 재생 쪽 데이터가 늦게 들어갈 수 있다 (`latency='low'` 는 TTS 에서 아직 시험하지 않은 설정) |
| R7 | **걸린 시간 로그**: `@quiet` 를 받은 시각과 재생이 멈춘 시각을 `time.monotonic()` 으로 남긴다. 평소에는 꺼 두고 `TTS_TIMING=1` 일 때만 `_tlog`(262행)로 남긴다. 버린 메시지·조각은 워커가 건너뛸 때 한 줄씩 로그로 남긴다 |
| R8 | **목표 시간**: `@quiet` 수신부터 소리가 멈출 때까지 0.1초 이내 |
| R9 | **기존 동작 유지**: `@quiet` 가 없으면 지금과 똑같이 동작한다. `.interrupt_trial` 의 "새 메시지가 올 때마다 끊기"는 넣지 않는다 (10-26 사용자가 "이상하다"고 해서 되돌린 동작). 빈 상태에서 `@quiet` 가 오면 로그만 남긴다 |
| R10 | **기존 코드는 지우지 않고 주석으로 남긴다.** 바꾸는 줄마다 바로 위에 `# [2026-10-30] 원본:` 으로 원래 줄을 주석 처리해 두고 그 아래에 새 줄을 쓴다. 새로 추가하는 부분은 `# [2026-10-30] @quiet` 주석으로 시작과 끝을 표시한다 |
| R11 | **되돌릴 수 있게**: 고치기 전에 `cp tts-kor-server.py tts-kor-server.py.before_quiet` 로 백업한다. 되돌리기는 `cp tts-kor-server.py.before_quiet tts-kor-server.py` 후 재시작(8장) |
| R12 | `audio_worker` 에서 `_playing` 에 값을 넣으므로 367행 `global` 에 `_playing` 을 추가한다 (빠뜨리면 UnboundLocalError) |
| R13 | 고치는 곳은 3장 방식에 필요한 곳(수신 루프의 `@quiet` 판별·enqueue 두 곳, `tts_worker`, `audio_worker`, 새 함수·변수)뿐이다. 다른 기능(볼륨 감시, 음악, NPU, 워밍업, 출력 장치 재선택, 종료 처리)은 건드리지 않는다 |

## 5. 구현 참고

`tts-kor-server.py.interrupt_trial` (10-26, 적용 안 한 시험본)의 `_gen`, `play_interruptible`, 워커의 세대 확인(312~321, 427행 부근)을 쓰면 된다.

시험본과 다른 점:
- 끊는 계기: "모든 새 메시지" → **`@quiet` 만**
- 큐 비우기(`_drain`): 하지 않음 (워커가 건너뜀)
- 재생 블록: 2048 → **1024**, `latency='low'`, 짧은 페이드, float32

구조 참고 (실제 코드에 맞춰 적용):

```python
# ── [2026-10-30] "@quiet": 쌓인 말 모두 버리고 말하던 문장 끊기 (Thor gdh "조용히 해") ──
# 메시지마다 받은 때의 세대 번호를 붙이고, @quiet 가 오면 번호만 올린다.
# 워커는 옛 세대 항목을 꺼내면 말하지 않고 버린다 (큐를 따로 비우지 않음).
_gen = 0
_playing = False               # 지금 TTS 를 말하는 중인지 (로그용)
_PLAY_BLOCK = 1024             # ≈ 21 ms. @quiet 를 이 단위로 확인
_FADE = 960                    # ≈ 20 ms 페이드아웃


def enqueue_tts(text, speed):
    message_queue.put((text, speed, _gen))


def play_interruptible(audio, gen):
    """블록 단위로 내보내다가 @quiet(세대 변경)가 오면 짧게 페이드아웃하고 멈춘다. 끝까지 재생했으면 True"""
    underflows = 0
    with sd.OutputStream(samplerate=TARGET_SR, channels=CHANNELS, dtype='float32',
                         blocksize=_PLAY_BLOCK, latency='low') as st:
        for i in range(0, len(audio), _PLAY_BLOCK):
            if gen != _gen:
                tail = audio[i:i + _FADE]
                fade = np.linspace(1.0, 0.0, len(tail), dtype=np.float32)
                st.write((tail * fade).astype(np.float32).reshape(-1, 1))
                return False
            if st.write(np.ascontiguousarray(audio[i:i + _PLAY_BLOCK], dtype=np.float32).reshape(-1, 1)):
                underflows += 1
    if underflows:   # 블록마다 남기면 로그가 넘치므로 조각당 한 번
        log(f"⚠️ 재생 underflow {underflows}회 ({len(audio) / TARGET_SR:.1f}초 조각)")
    return True

# 수신 루프 — @play / @stop 판별 바로 옆
#     if cmd == '@quiet':
#         _gen += 1
#         log(f"📥 수신: @quiet (세대 {_gen}, 말하던 중 {'예' if _playing else '아니오'}, "
#             f"대기 메시지 {message_queue.qsize()}건, 대기 조각 {audio_queue.qsize()}개)")
#         _tlog("quiet received")
#         continue
#     ...
#     enqueue_tts(text, speed)                 # 기존 message_queue.put((text, speed))
#
# tts_worker — message is None 확인(종료 신호)은 그대로 먼저, 그다음 text, speed, gen = message
#              gen != _gen 이면 log("🤫 옛 세대 메시지 버림 …") 후 건너뜀
#              for audio in text_to_speech(...): gen != _gen 이면 break; audio_queue.put((audio, gen))
# audio_worker — global 에 _playing 추가 (R12)
#                item = audio_queue.get(); item is None 이면 break; audio, gen = item
#                gen != _gen 이면 log("🤫 옛 세대 조각 버림") 후 건너뜀 (finally 의 task_done 은 그대로 불림)
#                _playing = True; 끝까지 재생 못 했으면 log("🤫 말하던 문장 끊음"), _tlog("play stopped")
#                finally: _playing = False
```

## 6. 기존 코드 보호 — 구현 전·후 확인

| 순서 | 확인 |
|---|---|
| 1 | 고치기 전: `md5sum tts-kor-server.py` 가 `11d7bb7d82a000369ffdde2c78772df1` 인지 확인. 다르면 고치지 말고 사용자에게 먼저 알린다 |
| 2 | 백업: `cp tts-kor-server.py tts-kor-server.py.before_quiet` (R11) |
| 3 | 고친 뒤: `.venv/bin/python3 -m py_compile tts-kor-server.py` 로 문법 확인 |
| 4 | `diff tts-kor-server.py.before_quiet tts-kor-server.py` 를 보고 R13 밖의 줄이 바뀌지 않았는지 확인. 지워진 원본 줄이 없는지(모두 주석으로 남았는지) 확인 |
| 5 | 재시작은 **사용자에게 확인한 뒤** 한다 (8장). 재시작 후 4070 포트가 열리고 시작 로그(모델 로드, NPU, 워밍업)가 평소와 같은지 확인 |
| 6 | **T7(기존 동작)을 가장 먼저** 시험한다. 평소와 다르면 그 자리에서 되돌린다 |
| 7 | 그다음 T1~T6 |
| 8 | 어느 단계든 서버가 안 뜨거나 평소와 다르게 동작하면 즉시 백업으로 되돌리고(R11) 재시작한 뒤 사용자에게 알린다 |

## 7. 시험

### 7.1 시험 도구

`~/TTS_Monitoring/quiet_test.py` 로 저장해 서버에서 실행한다. Thor 에서 실행하려면 주소를 `tcp://192.168.2.12:4070` 으로 준다.

```python
#!/usr/bin/env python3
"""@quiet 시험: python3 quiet_test.py <시험번호> [서버주소]"""
import struct, sys, time, zmq

addr = sys.argv[2] if len(sys.argv) > 2 else "tcp://127.0.0.1:4070"
sock = zmq.Context().socket(zmq.PUSH)
sock.connect(addr)
time.sleep(0.3)


def send(text, speed=1.5):
    sock.send(struct.pack('f', speed) + text.encode('utf-8'))
    print(f"{time.strftime('%H:%M:%S')} 보냄: {text}")


LONG = ["첫 번째 문장입니다. 지금부터 아주 긴 안내를 시작하겠습니다. 이 문장은 중간에 끊겨야 합니다.",
        "두 번째 문장입니다. 이 문장은 들리면 안 됩니다.",
        "세 번째 문장입니다. 이 문장도 들리면 안 됩니다."]

t = sys.argv[1]
if t == "1":    # 말하는 중 정지
    for s in LONG: send(s)
    time.sleep(3.0); send("@quiet")
elif t == "2":  # 합성 중 정지
    send(LONG[0]); time.sleep(0.2); send("@quiet")
elif t == "3":  # 정지 직후 도착한 말은 들려야 함
    for s in LONG: send(s)
    time.sleep(3.0); send("@quiet"); send("정지 뒤에 온 문장입니다. 이 문장은 들려야 합니다.")
elif t == "4":  # 빈 상태 정지
    send("@quiet"); time.sleep(1.0); send("정상 문장입니다.")
elif t == "5":  # 음악은 영향 없음
    send("@play gymnopedie"); time.sleep(2.0); send(LONG[0]); time.sleep(2.0); send("@quiet")
    time.sleep(3.0); send("@stop")
elif t == "6":  # 연달아 정지 (gdh 와 VLM 이 거의 동시에 보내는 경우)
    for s in LONG: send(s)
    time.sleep(2.0)
    for _ in range(3): send("@quiet")
    time.sleep(1.0); send("정상 문장입니다.")
elif t == "7":  # 기존 동작: @quiet 없이 순서대로 모두 말함 (끼어들기 없음)
    for s in LONG: send(s)
time.sleep(0.5)
```

### 7.2 기대 결과

| # | 시험 | 기대 결과 |
|---|---|---|
| T1 | 말하는 중 정지 | 첫 문장 도중 0.1초 안에 끊김. 2·3번 문장 안 들림 |
| T2 | 합성 중 정지 | 아무 소리도 안 나거나 첫 조각 일부만 나고 끊김 |
| T3 | 정지 직후 도착 | "정지 뒤에 온 문장입니다…"는 들림 |
| T4 | 빈 상태 정지 | 오류 없음, "정상 문장입니다" 들림 |
| T5 | 음악 | `@quiet` 뒤에도 음악 계속, 말만 끊김. `@stop` 으로 음악 정지 |
| T6 | 연달아 정지 | 오류 없음, 마지막 "정상 문장입니다" 들림 |
| T7 | 기존 동작 | 세 문장 모두 순서대로, 끊김 잡음 없이 들림 (지금과 같음) |

- `TTS_TIMING=1` 로 띄워 T1 의 정지 시간을 잰다.
- T1·T7 은 로봇 부하가 걸린 상태에서도 underflow 잡음이 없는지 확인한다. **부하 시험은 로봇을 멈춘 상태에서, 사용자에게 먼저 물어본 뒤** 한다.
- 확인할 로그: `tts_server.log` 의 `📥 수신: @quiet`, `🤫 …`, `⚠️ 재생 underflow`

## 8. 재시작

- `~/TTS_Monitoring/npu/restart_server.sh` 로 재시작한다 (그래픽 세션 필요, 보통 약 10초).
- 재시작하는 동안 로봇 안내 음성이 나오지 않으므로, **재시작 전에 사용자에게 확인**한다.
- 재시작이 늦으면 SSD 온도부터 확인한다.

## 9. 완료 보고

`~/TTS_Monitoring/docs/tts_quiet_report_261030.md` 에 아래 내용을 저장한다 (Thor 쪽이 읽어 감).

1. 바꾼 파일과 바뀐 내용 (diff)
2. T1~T7 결과와 `@quiet` 수신부터 소리가 멈출 때까지 걸린 시간
3. underflow 발생 여부 (부하 시험을 했다면 그 결과)
4. 서버 재시작 시각 — 이 시각 이후 Thor 쪽에서 "조용히 해"를 켠다
5. 이 문서와 다르게 한 점과 그 이유
6. 6장 확인 결과 (md5, py_compile, diff 검토)

## 10. 범위 밖

- Thor 쪽 음성 인식·image_ai_node·VLM 수정 (Thor 에서 함)
- 정지 후 일정 시간 동안 메시지를 무시하는 기능 (넣지 않기로 함)
- 음악·효과음 정지 (`@stop` 그대로)

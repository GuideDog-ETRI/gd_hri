#!/usr/bin/env python3
"""ZMQ test input program for llama-guidedog-cli.

Plays the cam_server side of the guidedog VLM server:

  * cam_server side : XPUB bind tcp://*:5555  -> sends [header, BGR image, question]

The answer goes to the real TTS server (ZMQ PULL on tcp://*:4070), which is NOT part of this
program. For a quick check without a TTS server, --tts-sink binds that PULL socket here and
prints the speech segments ([float32 rate][utf-8 text]) as they arrive.

Wire format (same as cam_server.py of the on-device-guidedog stack):
  header  = struct.pack('iiidff', h, w, c, timestamp, goal_x, goal_y)   # 32 bytes, native alignment
  image   = h*w*c raw bytes, BGR order (OpenCV layout)
  prompt  = optional utf-8 text (3rd part). Empty / missing -> server uses its -p prompt.

Interactive mode (default when --image is not given, or with -i): one line = "image path / question".
The header goal (goal_x, goal_y) is taken from "(x, y)" numbers in the question, else from a
"_gx_gy_" pattern in the file name, else from /goal (default 0.5 0.5).

  python3 tools/mtmd/guidedog-zmq-test.py
  > test_0.51096_0.63526_1280.jpg / 이 장면에서 보행자가 주의해야 할 점은 무엇인가요?
  > test_0.51096_0.63526_1280.jpg / 상세
  > test.jpg / [일반] 이미지를 분석하여 목표 위치 (0.5, 0.55)를 기반으로 시각 장애인을 위한 간단한 도보 안내를 제공해 주세요.

Batch mode:
  python3 tools/mtmd/guidedog-zmq-test.py --image test_0.51096_0.63526_1280.jpg --mode all
  python3 tools/mtmd/guidedog-zmq-test.py --image a.jpg --goal 0.5 0.55 --prompt "..."
  python3 tools/mtmd/guidedog-zmq-test.py --image a.jpg --goal 0.5 0.55 --no-prompt
"""
import argparse
import os
import re
import struct
import sys
import time

import numpy as np
import zmq
from PIL import Image

try:
    import readline  # noqa: F401  (line editing + history for input())
except ImportError:
    pass

# question templates from the 2026-10-01 e-mail (RFM lab). {x}/{y} are filled with the goal position.
PROMPTS = {
    "요약": "[요약] 이미지를 분석하여 목표 위치 ({x}, {y})를 기반으로 시각 장애인을 위한 간단한 도보 안내를 제공해 주세요.",
    "일반": "[일반] 이미지를 분석하여 목표 위치 ({x}, {y})를 기반으로 시각 장애인을 위한 간단한 도보 안내를 제공해 주세요.",
    "상세": "[상세] 이미지를 분석하여 목표 위치 ({x}, {y})를 기반으로 장애물의 방향과 위치 정보를 포함한 시각 장애인을 위한 간단한 도보 안내를 제공해 주세요.",
    "자유": "이 장면에서 보행자가 주의해야 할 점은 무엇인가요?",
}
MODE_ORDER = ["요약", "일반", "상세", "자유"]
MODE_ALIASES = {"요약": "요약", "[요약]": "요약", "일반": "일반", "[일반]": "일반",
                "상세": "상세", "[상세]": "상세", "자유": "자유", "free": "자유"}
IMG_EXT = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}

HELP = """\
입력: 이미지경로 / 질문
  예) test_0.51096_0.63526_1280.jpg / 이 장면에서 보행자가 주의해야 할 점은 무엇인가요?
      test.jpg / 상세            (요약 | 일반 | 상세 | 자유 = 메일 양식의 기본 질문)
      test.jpg / [일반] 이미지를 분석하여 목표 위치 (0.5, 0.55)를 기반으로 시각 장애인을 위한 간단한 도보 안내를 제공해 주세요.
  질문만 입력하면 직전 이미지로 보냅니다.
goal (헤더의 goal_x, goal_y): 질문 안의 (x, y) 숫자 > 파일명의 _gx_gy_ > /goal 값 순으로 정합니다.
명령: /goal <x> <y>    /help    /quit (Ctrl-D)
답변은 TTS 서버(4070)로 갑니다. TTS 서버가 없으면 --tts-sink 로 실행해 여기서 확인합니다.
"""
SEP_RE = re.compile(r"\s+/\s+")  # "이미지경로 / 질문" (paths contain '/', so the separator needs spaces)
GOAL_RE = re.compile(r"\(\s*(\d+(?:\.\d+)?)\s*,\s*(\d+(?:\.\d+)?)\s*\)")


def goal_from_filename(path):
    # test_0.51096_0.63526_1280.jpg -> (0.51096, 0.63526)
    m = re.search(r"_(\d+\.\d+)_(\d+\.\d+)_", os.path.basename(path))
    return (float(m.group(1)), float(m.group(2))) if m else None


def goal_from_text(text):
    # "... 목표 위치 (0.5, 0.55)를 ..." -> (0.5, 0.55); "(x, y)" placeholder -> None
    m = GOAL_RE.search(text or "")
    return (float(m.group(1)), float(m.group(2))) if m else None


def load_bgr(path):
    img = Image.open(path).convert("RGB")
    rgb = np.asarray(img, dtype=np.uint8)
    return np.ascontiguousarray(rgb[:, :, ::-1])


def fmt_goal(goal, round2):
    gx, gy = goal
    if round2:
        # the SAVLM prompts were trained with 2-decimal goals, e.g. "(0.5, 0.55)"
        gx, gy = round(gx, 2), round(gy, 2)
    return gx, gy


class GuidedogClient:
    """cam_server side (XPUB); optionally also the TTS sink (PULL) for local checks."""

    def __init__(self, pub_addr, pull_addr=None):
        self.ctx = zmq.Context()
        # XPUB behaves like PUB for sending but also hands us the subscription message that the
        # server's SUB socket sends on connect, so we can start before the server and avoid the
        # PUB/SUB slow-joiner problem (a frame published before the SUB joins is silently dropped).
        self.pub = self.ctx.socket(zmq.XPUB)
        self.pub.bind(pub_addr)
        self.pull = None
        if pull_addr:
            self.pull = self.ctx.socket(zmq.PULL)
            self.pull.bind(pull_addr)
        self.n_sub = 0

    def _drain_sub_events(self):
        while self.pub.poll(0):
            ev = self.pub.recv()
            self.n_sub = max(0, self.n_sub + (1 if ev[:1] == b"\x01" else -1))

    def wait_for_server(self, timeout_s):
        """block until at least one SUB is connected (timeout_s=None -> forever). True if connected."""
        self._drain_sub_events()
        if self.n_sub > 0:
            return True
        deadline = None if timeout_s is None else time.time() + timeout_s
        while self.n_sub == 0:
            remaining = None if deadline is None else deadline - time.time()
            if remaining is not None and remaining <= 0:
                return False
            if not self.pub.poll(None if remaining is None else int(remaining * 1000)):
                return False
            self._drain_sub_events()
        time.sleep(0.2)  # let the freshly joined pipe settle before the first frame
        return True

    def send(self, bgr, gx, gy, prompt):
        h, w, c = bgr.shape
        header = struct.pack("iiidff", h, w, c, time.time(), gx, gy)
        assert len(header) == 32, len(header)
        parts = [header, bgr.tobytes()]
        if prompt is not None:
            parts.append(prompt.encode("utf-8"))
        self.pub.send_multipart(parts)
        return time.time()

    def recv_answer(self, t_send, idle_s, timeout_s):
        """Collect TTS messages until `idle_s` seconds of silence after the first message
        (or `timeout_s` since send with nothing received). Returns list of (t_rel, rate, text)."""
        poller = zmq.Poller()
        poller.register(self.pull, zmq.POLLIN)
        msgs = []
        t_last = None
        while True:
            now = time.time()
            wait = (t_last + idle_s - now) if t_last else (t_send + timeout_s - now)
            if wait <= 0:
                break
            if poller.poll(int(wait * 1000)):
                raw = self.pull.recv()
                t_last = time.time()
                if len(raw) < 4:
                    print(f"  [warn] short message ({len(raw)} bytes)")
                    continue
                (rate,) = struct.unpack("<f", raw[:4])
                text = raw[4:].decode("utf-8", errors="replace")
                msgs.append((t_last - t_send, rate, text))
                print(f"  [+{t_last - t_send:6.2f}s] rate={rate:.1f}  {text}")
                sys.stdout.flush()
        return msgs


def ask(client, bgr, goal, prompt, args, label=""):
    """send one frame (+ optional question) and print the streamed answer. Returns the messages."""
    gx, gy = goal
    h, w, _ = bgr.shape
    print(f"=== {label}{'(질문 없음, 서버 -p 템플릿)' if prompt is None else prompt}")
    print(f"    frame {w}x{h}, goal=({gx}, {gy})")
    sys.stdout.flush()
    if not client.wait_for_server(args.sub_timeout if args.sub_timeout > 0 else None):
        print("  !! VLM 서버가 연결되어 있지 않습니다 (SUB 없음) - 전송 취소")
        return []
    t_send = client.send(bgr, gx, gy, prompt)
    if client.pull is None:
        print(f"  -> 전송 완료 {time.strftime('%H:%M:%S')} (답변은 TTS 서버 / VLM 서버 로그에서 확인)")
        return []
    msgs = client.recv_answer(t_send, args.idle, args.timeout)
    if not msgs:
        print("  !! 응답 없음 (서버 다운 / 미구독 / 빈 답변)")
    else:
        full = " ".join(m[2].strip() for m in msgs)
        has_tag = "<" in full and ">" in full
        print(f"  -> {len(msgs)}개 메시지, 첫 {msgs[0][0]:.2f}s, 끝 {msgs[-1][0]:.2f}s, "
              f"xml 태그 잔존: {'있음' if has_tag else '없음'}")
    return msgs


# ----------------------------------------------------------------------------- interactive

def interactive(client, args, image, goal):
    cur = {"image": None, "bgr": None}
    manual_goal = goal  # --goal / file name at start, changed with /goal

    def set_image(path):
        path = os.path.expanduser(path)
        if os.path.splitext(path)[1].lower() not in IMG_EXT:
            print(f"  !! 이미지 파일이 아닙니다: {path}  (입력 형식: 이미지경로 / 질문)")
            return False
        try:
            bgr = load_bgr(path)
        except Exception as e:  # noqa: BLE001
            print(f"  !! 이미지를 열 수 없습니다: {path} ({e})")
            return False
        cur["image"], cur["bgr"] = path, bgr
        return True

    def current_goal(question=None):
        g = goal_from_text(question)
        src = "질문"
        if g is None and cur["image"]:
            g, src = goal_from_filename(cur["image"]), "파일명"
        if g is None:
            g, src = manual_goal, "/goal"
        return fmt_goal(g, not args.raw_goal), src

    print(HELP)
    if image:
        set_image(image)
    while True:
        img_name = os.path.basename(cur["image"]) if cur["image"] else "이미지 없음"
        g, _ = current_goal()
        try:
            line = input(f"[{img_name} | goal=({g[0]}, {g[1]})] > ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\n종료")
            return 0
        if not line:
            continue

        # a command starts with '/', but so does an absolute path: "이미지경로 / 질문" or "*.jpg" wins
        is_cmd = line.startswith("/") and not SEP_RE.search(line) and os.path.splitext(line)[1].lower() not in IMG_EXT
        if is_cmd:
            cmd, _, arg = line.partition(" ")
            cmd, arg = cmd.lower(), arg.strip()
            if cmd in ("/quit", "/exit", "/q"):
                print("종료")
                return 0
            elif cmd in ("/help", "/h", "/?"):
                print(HELP)
            elif cmd == "/goal":
                try:
                    gx, gy = (float(v) for v in arg.replace(",", " ").split())
                    manual_goal = (gx, gy)
                    print(f"  goal=({gx}, {gy})")
                except ValueError:
                    print("  사용법: /goal <x> <y>   예) /goal 0.5 0.55")
            else:
                print(f"  알 수 없는 명령: {cmd}  (/help)")
            continue

        # "이미지경로 / 질문"  |  "질문" (previous image)  |  "이미지경로" (just switch the image)
        parts = SEP_RE.split(line, maxsplit=1)
        if len(parts) == 2:
            path, question = parts[0].strip(), parts[1].strip()
        elif os.path.splitext(line)[1].lower() in IMG_EXT:
            path, question = line, ""
        else:
            path, question = None, line
        if path is not None and not set_image(path):
            continue
        if cur["bgr"] is None:
            print("  !! 이미지가 없습니다. '이미지경로 / 질문' 으로 입력하세요")
            continue
        if not question:
            h, w, _ = cur["bgr"].shape
            print(f"  이미지 설정: {cur['image']} ({w}x{h}) - 질문을 입력하세요 (요약/일반/상세/자유 또는 한글 질문)")
            continue

        g, src = current_goal(question)
        if question in MODE_ALIASES:
            question = PROMPTS[MODE_ALIASES[question]].format(x=g[0], y=g[1])
        print(f"    goal=({g[0]}, {g[1]}) <- {src}")
        try:
            ask(client, cur["bgr"], g, question, args)
        except KeyboardInterrupt:
            print("\n  (응답 대기 취소)")


# ----------------------------------------------------------------------------- batch

def batch(client, args, image, goal):
    bgr = load_bgr(image)
    gx, gy = fmt_goal(goal, not args.raw_goal)
    h, w, _ = bgr.shape
    print(f"image {image} ({w}x{h}), goal=({gx}, {gy})")

    if args.no_prompt:
        tests = [("(server -p)", None)]
    elif args.prompt:
        tests = [("custom", args.prompt)]
    else:
        modes = MODE_ORDER if args.mode == "all" else [m.strip() for m in args.mode.split(",")]
        for m in modes:
            if m not in PROMPTS:
                print(f"unknown mode '{m}' (choose from {', '.join(PROMPTS)}, all)")
                return 2
        tests = [(m, PROMPTS[m].format(x=gx, y=gy)) for m in modes]

    summary = []
    for rep in range(args.repeat):
        for idx, (name, prompt) in enumerate(tests):
            print()
            msgs = ask(client, bgr, (gx, gy), prompt, args, label=f"[{name}] ")
            full = " ".join(m[2].strip() for m in msgs)
            summary.append((rep, name, len(msgs), msgs[0][0] if msgs else None, msgs[-1][0] if msgs else None, full))
            if client.pull is None and (rep < args.repeat - 1 or idx < len(tests) - 1):
                time.sleep(args.gap)  # no sink to tell us when the answer ended: pace the frames

    if client.pull is None:
        print(f"\n{len(summary)}개 프레임 전송 완료 (답변은 TTS 서버에서 확인, 여기서 보려면 --tts-sink)")
        return 0

    print("\n===== summary =====")
    for rep, name, n, first, last, full in summary:
        tag = f"rep{rep} " if args.repeat > 1 else ""
        if n == 0:
            print(f"{tag}[{name}] FAILED (no message)")
        else:
            print(f"{tag}[{name}] {n} msg, first {first:.2f}s, last {last:.2f}s | "
                  f"{full[:160]}{'...' if len(full) > 160 else ''}")
    return 1 if any(s[2] == 0 for s in summary) else 0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--image", help="image file sent as the camera frame (omit for interactive mode)")
    ap.add_argument("-i", "--interactive", action="store_true", help="interactive mode even when --image is given")
    ap.add_argument("--goal", nargs=2, type=float, metavar=("GX", "GY"),
                    help="normalized goal position; default: parsed from file name, else 0.5 0.5")
    ap.add_argument("--mode", default="all", help="batch: 요약 | 일반 | 상세 | 자유 | all (default) | comma list")
    ap.add_argument("--prompt", help="batch: send this exact question instead of --mode templates")
    ap.add_argument("--no-prompt", action="store_true", help="batch: send header+image only (server uses its -p prompt)")
    ap.add_argument("--raw-goal", action="store_true",
                    help="send the goal with full precision; default rounds to 2 decimals like the training data")
    ap.add_argument("--pub", default="tcp://*:5555", help="bind address for the frame XPUB socket")
    ap.add_argument("--tts-sink", action="store_true",
                    help="no TTS server running: bind the TTS PULL socket here and print the answers")
    ap.add_argument("--pull", default="tcp://*:4070", help="bind address for the TTS PULL socket (--tts-sink)")
    ap.add_argument("--idle", type=float, default=3.0, help="--tts-sink: seconds of silence that end one answer")
    ap.add_argument("--timeout", type=float, default=180.0, help="--tts-sink: max seconds to wait for the first message")
    ap.add_argument("--gap", type=float, default=10.0, help="batch without --tts-sink: seconds between frames")
    ap.add_argument("--sub-timeout", type=float, default=600.0,
                    help="seconds to wait for the VLM server to subscribe before sending (0 = wait forever)")
    ap.add_argument("--repeat", type=int, default=1, help="batch: repeat the whole prompt list N times")
    args = ap.parse_args()

    image = os.path.expanduser(args.image) if args.image else None
    if image and not os.path.isfile(image):
        ap.error(f"image not found: {image}")
    goal = tuple(args.goal) if args.goal else ((goal_from_filename(image) if image else None) or (0.5, 0.5))

    client = GuidedogClient(args.pub, args.pull if args.tts_sink else None)
    print(f"frame XPUB bound {args.pub}" + (f" | TTS PULL bound {args.pull} (--tts-sink)" if args.tts_sink
                                            else " | 답변은 TTS 서버(4070)로 전달됩니다"))

    if args.interactive or not image:
        return interactive(client, args, image, goal)
    return batch(client, args, image, goal)


if __name__ == "__main__":
    sys.exit(main())

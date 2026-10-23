GDH (GuideDog HRI) module packages
==================================

# 1. 소개
[GuideDog](https://github.com/GuideDog-ETRI) 과제의 HRI파트 설치 문서이다. gdh_package (static object detection service), gdh_image (test image publisher), gdh_speech_audio (STT와 TTS), gdh_vlm_client (SA-VLM의 client 파트)로 구성되어있다.

# 2. 설치
## 2.1. gdh_packages, gdh_speech_audio, gdh_image, gdh_vlm_client
- Download GDH packages from [github](https://github.com/GuideDog-ETRI/gd_hri).
    ```bash
    git clone https://github.com/GuideDog-ETRI/gd_hri .
    ```

- If you have a tensorflow, we may meet un error related to tensorflow version, run below, first.
    ```
    pip uninstall tensorflow
    ```

- pyaudio 설치
    ```
    sudo apt update
    sudo apt install portaudio19-dev
    pip3 install pyaudio
    ```
    
- pulseaudio 설치와 데몬 실행
    ```
    sudo apt-get install pulseaudio
    sudo apt-get install pulseaudio-module-bluetooth
    sudo pulseaudio --start
    ```
    
- OpenAI Whisper관련 모듈 설치
    ```
    pip install SpeechRecognition
    pip install git+https://github.com/openai/whisper.git
    pip install numba==0.61
    ```

- Download gd_ifc_pkg (interfaces for all gd modules) from [github](https://github.com/GuideDog-ETRI/gd_ifc_pkg).
    ```bash
    cd src
    git clone https://github.com/GuideDog-ETRI/gd_ifc_pkg    
    ```

    ```bash
    sudo apt-get update
    sudo apt-get install ros-humble-sensor-msgs
    sudo apt-get install ros-humble-nav-msgs
    sudo apt-get install ros-humble-grid-map-msgs
    sudo apt-get install ros-humble-vision-msgs
    ```
    ```
    cd ..
    ```

- To use yolo-v8.
    ```bash    
    pip install ultralytics
    pip install numpy
    ```

- (No more use) To use depth estimation
    ```bash
    pip install transformers
    ```

- Download Yolo models (gddemo_v2.1 version) from [Google drive](https://drive.google.com/drive/folders/1DFF6rFE7NYYMgBKvXmN59T1wD3KD05Tb?usp=sharing) or [GD Server](\\129.254.81.123\GuideDog_NAS\50_DB\gdh_src\models) and place in the folder /models.

- Build
    ```bash
    ./0build.sh
    ```

  
## 2.2. Option 설정
- models/gdh_config.yaml 파일에 다양한 옵션 설정이 가능함


# 3. 동작
## 3.1. gdh_package (정적객체검출기) 파트
- Open three new terminals by Ctrl + Alt + T for (A) dummy photo publisher, (B) GDH node, and (C) toy test client

- (A) 이미지를 publish하기위해 리코세타를 설치하지않고 녹화된 영상을 재생하겠다면, [다음 링크](https://drive.google.com/file/d/18xELEj7PeVmdU_xKqT7OH1pQPjSQ95zT/view?usp=drive_link)에서 파일을 받고 rosbag/rosbag2_2024_07_11-16_34_43 폴더로 복사 후, 다음을 실행한다. ros2 topic list 입력시 /image_raw가 있다면 성공.
    ```
    cd gd_hri/rosbag
	./play.bash

	ros2 topic list    # /theta/image_raw/compressed 확인
	ros2 topic info /theta/image_raw/compressed    # Type과 Publisher 확인
	ros2 topic echo /theta/image_raw/compressed    # 숫자 변하는지 확인
    ```

- (B) Run a GDH node (receiving images and processing HRI functions). 정상적이라면 이미지를 받은 알림과 GDHHeartBeat msg의 출력을 볼 수 있음.
    ```bash
    ./1run.sh
    ./1run_detector.sh  # 만약, 검출기만 실행하고자하는 경우
    # models/gdh_config.yaml 파일 내에서 image topic 이름, decompress여부, rectify 여부 선택가능
    ```

- (C) Request services
    ```bash
    . install/setup.bash

    # start detector
    ros2 service call /GDH_start_detect gd_ifc_pkg/srv/GDHStartDetectObject "{object_types: 10}"

    # stop detector
    ros2 service call /GDH_stop_detect gd_ifc_pkg/srv/GDHStopDetectObject
    ```
    * Results are saved in the GDH folder.

- (D) 동작확인
    ```bash
    rqt
    ```

- (E) 결과 녹화
    ```bash
    ./2run_image_saver.sh
	# models/gdh_config.yaml 파일 내에서 save 파일명 수정가능
    ```

## 3.2. gdh_speech_audio (STT/TTS) 파트
- Open three new terminals for GDH node, toy client for sending speech codes, toy server for receiving command id.
- (A) Run GDH node
    ```bash
    ./1run.sh
    ```

- (B) Echo messages from voice command
    ```
    ros2 topic list
    ros2 topic echo /GDH_user_cmd
    ```

- (C) Run a toy client for sending speech codes
    ```bash
    . install/setup.bash
    ros2 run gdh_package client_code_send
    ```

- [2026-10-23] 샥즈(OpenComm2) 음성 인식 불안정 수정
    * 증상: 샥즈는 연결돼 있는데 음성 인식이 됐다 안 됐다 함. 아예 안 되는 구간도 있음
    * 실측 결과
        - 샥즈 녹음은 정상: 말소리 rms 3000~4500, 주변 소음 rms 약 40, 클리핑 없음
        - 녹음 소스 감시(`_mic_guard_loop`)가 `pactl` 응답 지연(3~5초, timeout 3초)을 "샥즈 아님"으로 처리해 시간의 약 2/3 동안 음성을 버림. 시험한 4문장 모두 버려짐
            + `pactl`이 느린 원인: NoMachine 접속 시 NoMachine용 WirePlumber가 하나 더 떠서 메인 WirePlumber와 사운드카드를 번갈아 잡음 (`journalctl --user -u wireplumber`의 `ReserveDevice1.Audio0/1` 반복)
        - 상태가 바뀔 때마다 주변 소음을 다시 보정해, 말하는 중에 보정되면 기준(energy_threshold)이 2000~3500으로 튐
        - Whisper(large-v3-turbo) 자체는 정확함(4문장 중 3개). "에디야"가 "애리야"로 들리고, 무음·잡음에서 "감사합니다", "다음 영상에서 만나요" 같은 문장을 지어냄
    * 수정한 곳 (`src/gdh_speech_audio/gdh_speech_audio/speech_to_cmd_by_whisper.py`, 기존 코드는 `[2026-10-23] 원본` 주석으로 남김)
        | 위치 | 수정 내용 |
        |---|---|
        | `PACTL_TIMEOUT`, `pactl_json()`, `_pin_to_shokz()` | `pactl` timeout 3초 → 10초 |
        | `_mic_guard_loop()` | `pactl`이 실패하면 음성을 버리지 않고 직전 상태 유지 |
        | `_listen_loop()` | 주변 소음 보정 결과에 상한(`MAX_ENERGY_THRESHOLD`) 적용, 말소리마다 `[VOICE]` 진단 로그 |
        | `transcribe_once_base()` | Whisper에 `language`, `initial_prompt` 지정, 확신도(`avg_logprob`)가 `MIN_AVG_LOGPROB`보다 낮으면 버림, `[WHISPER]` 진단 로그 |
        | `_listen_and_publish()` | 명령 매칭 전 "애리야/에리야/애리아/에리아" → "에디야", "지금의" → "지금" 보정 추가 |
    * 추가한 설정 (`models/gdh_config.yaml`의 `openai_whisper`, 없으면 코드 기본값 사용)
        | 키 | 값 | 설명 |
        |---|---|---|
        | `LANGUAGE` | `"ko"` | Whisper 인식 언어 (`null`이면 자동 판별) |
        | `INITIAL_PROMPT` | `"에디야, 멈춰. 에디야, 가자. 지금 상황 어때? 신호등 찾아."` | 호출어·명령 힌트 문장 |
        | `MIN_AVG_LOGPROB` | `-1.0` | 실측: 실제 말 -0.04~-0.27, 지어낸 문장 -2.6~-7.9 |
        | `MAX_ENERGY_THRESHOLD` | `1000` | 주변 소음 보정 결과 상한 (처음 1500 → 재연결 직후 1500으로 잘렸는데 말소리 rms가 1100~1400이라 1000으로 낮춤, 평소 보정값 440~460) |
        | `RECORD_TIMEOUT` (기존 키) | `2.0` → `4.0` | 한 문장 최대 녹음 길이. 2초 넘는 명령이 잘려 두 조각으로 나뉘어 매칭 실패하던 문제 ("지금 지하..." + "멈춰 입구 찾아") |
    * 확인 방법: 노드 로그(`~/.ros/log/python3_*.log`)에서
        - `[VOICE] 1.6s rms=3000 (기준 460) -> Whisper`: 말소리가 Whisper로 넘어감 (`-> 버림`이면 샥즈 녹음 상태가 아니라서 버림, `RECORD_TIMEOUT 에서 잘림`이면 말이 `RECORD_TIMEOUT`(4초)을 넘어 잘림)
        - `[WHISPER] 1.6s -> “지금 상황 어때?” (avg_logprob=-0.1, 0.16s)`: 받아쓰기 결과 (`신뢰도 낮음 - 버림`이면 지어낸 문장으로 보고 버림)
        - `[MIC] pactl 확인 실패 - 직전 상태 유지`: 오디오 서버가 느림 (음성은 버리지 않음)
    * 같이 바꾼 Thor 시스템 설정 (오디오 서버가 느렸던 원인 제거, NoMachine으로는 소리를 전혀 쓰지 않음)
        | 파일 | 수정 내용 |
        |---|---|
        | `/usr/NX/etc/node.cfg` | `AudioInterface disabled`, `EnableAudio 0` (원본 줄은 주석, 백업 `node.cfg.bak_audio_20261023_114607`). 어느 PC에서 접속하든 새 세션은 NoMachine용 PipeWire·WirePlumber를 띄우지 않음 |
        | `~/.config/wireplumber/main.lua.d/51-nx-session-no-devices.lua` | NoMachine이 띄운 WirePlumber(`PIPEWIRE_REMOTE=pipewire-NNNN`)는 사운드카드·카메라 관리 안 함 (안전장치) |
        | `~/.config/wireplumber/bluetooth.lua.d/51-nx-session-no-bluetooth.lua` | 같은 조건에서 블루투스 관리 안 함. BlueZ에 A2DP 연결 통로가 중복 등록돼 샥즈 재연결 때 `Endpoint replied with an error: NoReply` 후 끊겼다 다시 붙던 문제 |
        - 메인 WirePlumber(`systemd --user`, `PIPEWIRE_REMOTE` 없음)에는 영향 없음
        - 적용 결과: `pactl` 응답 2.8~10초 → 12~18ms, 사운드카드 쟁탈(`ReserveDevice1.Audio0/1`) 1분 19~34회 → 0회
        - `node.cfg`는 세션이 시작될 때 읽으므로, 이전에 만들어진 NoMachine 세션은 종료(또는 재부팅) 후 완전히 적용됨

## 3.3. GDH Heartbeat status
- (A) Run GDH node
    ```bash
    conda activate use_gopro
    cd ~/Desktop/gdh
    . install/setup.bash
    ros2 run gdh_package service
    ```

- (B) Echo status
    ```
    ros2 topic list
    ros2 topic echo /GDH_status
    ```

## 3.4. gdh_vlm_client (SA-VLM) 파트
- 별도의 docker로 동작하는 TTS와 SA-VLM 고속화 버전을 실행한다.
- 이후 음성 명령으로 지금상황어때, 왜 멈췄어를 물어보면 된다.
    

# 4. 오류 처리
- if gd_ifc_pkg makes new errors, install belows
    ```
    sudo apt-get update
    sudo apt-get install ros-humble-sensor-msgs 
    sudo apt-get install ros-humble-nav-msgs 
    sudo apt-get install ros-humble-grid-map-msgs ros-humble-vision-msgs
    ```

- OSError: Invalid sample rate
    1. pulseaudio 설치    
    ```
    sudo apt-get install pulseaudio
    sudo apt-get install pulseaudio-module-bluetooth
    ```

    2. pulseaudio 데몬 실행 (optional)
    ```
    pulseaudio --start
    ```

    3. ~/.asoundrc 파일 생성 혹은 변경
    ```
    pcm.!default {
        type pulse
    }

    ctl.!default {
        type pulse
    }
    ```

    4. docker run 코드에 다음 추가
    ```
    docker run -it --rm \
        -e PULSE_SERVER=unix:${XDG_RUNTIME_DIR}/pulse/native \
        -v ${XDG_RUNTIME_DIR}/pulse/native:${XDG_RUNTIME_DIR}/pulse/native \
        -v ~/.config/pulse/cookie:/root/.config/pulse/cookie \
        your-docker-image
    ```

- 마이크 볼륨 컨트롤러가 안보이면,
    ```
    sudo apt install pavucontrol
    ```

- NVIDIA Orin에서 설치시에 블루투스 동작에서 문제가 발생하는데 이와 관련된 해결책은 아래와 같음
    * [명령어 업데이트 방식](https://forums.developer.nvidia.com/t/nvidia-jetson-xavier-nx-bluetooth-connection-issue/156351/31) -> 시도했지만 해결 안됨
    * [Jetpack update 방식](https://forums.developer.nvidia.com/t/bluetooth-connectivity-issues-with-jetson-orin-nano/290484)
    
- py360convert.ep()함수로 rectified 된 이미지의 크기가 정사각형이라면, py360convert의 라이브러리 업데이트가 필요 (>= 1.0.2)
    ```
    pip show py360convert
    pip install --upgrade py360convert
    ```
 

<!-- - 사운드 카드 미인식 혹은 설정 오류
    ```
    ALSA lib confmisc.c:855:(parse_card) cannot find card '0'
    ALSA lib conf.c:5178:(_snd_config_evaluate) function snd_func_card_inum returned error: No such file or directory
    ```
    리눅스 시스템에 로그인 필요
    아래 명령으로 사운드카드 인식상태 확인
    ```
    aplay -l
    ``` -->

# 5. SA-VLM on Orin 설치 (안되는 경우)
    ```bash
    cd gdh_savlm/docker
    # sudo docker container ls -a
    ./start.sh
    ```

    docker 안에서,
    ```bash
    pip install pygame
    pip install openai
    
    pip uninstall qwen-vl-utils
    pip install qwen-vl-utils[decord]
    
    apt update
    apt install ffmpeg

    pip uninstall vllm
    pip install vllm==0.7.4+cu126
    ```
    
    import vllm._C  # noqa
    ImportError: /usr/local/lib/python3.10/dist-packages/vllm/_C.abi3.so: undefined symbol: _ZNK3c1011StorageImpl27throw_data_ptr_access_errorEv
    에러가 나옵니다. vllm 동작시 필요한 library가 없어서 발생하는 에러.     
    ```
    git clone https://github.com/vllm-project/vllm.git
    cd vllm
    VLLM_USE_PRECOMPILED=1 pip install --editable .    
    ```
    
    VLLM이 동작하기위한 관련 모듈이 업데이트 됨. 이후 다시 설치
    ```
    pip install vllm==0.7.4+cu126
    ```

    orin h/w 사용량 체크
    ```
    jtop
    ```





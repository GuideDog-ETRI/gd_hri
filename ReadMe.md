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





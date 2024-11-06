GDH (GuideDog HRI) module packages
==================================

# 1. 소개
[GuideDog](https://github.com/GuideDog-ETRI) 과제의 HRI파트 설치 문서이다. 리코세타 설치 (옵션), GDH package 설치로 나누어져 있다.

# 2. 설치
## 2.1. 리코세타(ricoh thetaz1)

- 리코세타가 설치되었으면 skip가능
- 리코세타 설치없이 녹화된 영상을 사용하겠다면 역시 skip가능

2.1.1. 리코세타 연결
- 리코세타 카메라를 USB에 연결하고(초록불 충전중), 전원 스위치를 켠다(파란불). 창에 그림이 나오면, 모드 버튼을 눌러서 모드를 '캠코더+LIVE'로 바꾼다.

2.1.2. Install driver with docker
- To share topics between hostpc and docker container, add below code in ~/.bashrc in hostpc
    ```bash
    export ROS_DOMAIN_ID=7  # max 232, 번호는 자유롭게 설정. 통신하려는 장치끼리 같은 번호 써야 함.
    export ROS_LOCALHOST_ONLY=1 # Topic will not go out of the PC. (It's ok between docker containers)
    ```

- [gd_theta_driver](https://github.com/GuideDog-ETRI/gd_theta_driver)의 without docker를 따라서 설치
- 다음과 유사하게 나와야 함
    ```bash
    [INFO] [1720663252.803293205] [theta_driver]: Initializing
    [INFO] [1720663252.967076961] [theta_driver]: index: 0
    [INFO] [1720663252.967143354] [theta_driver]: product: RICOH THETA Z1
    [INFO] [1720663252.967154849] [theta_driver]: serial: 123456789
    [INFO] [1720663253.079030654] [theta_driver]: Start streaming
    ```

- 아니라면 다음을 명시적으로 실행해 볼 것
    ``` bash
    ros2 run theta_driver theta_driver_node
    ```

- toplic /theta/image_raw/compressed 을 확인
    ```bash
    ros2 topic list
    ```

## 2.2. GDH packages
- Assume all module is installed in conda env 'use_gopro'
    ```bash
    conda activate use_gopro
    ```

- Make a workspace
    ```bash
    mkdir ~/Desktop/gdh
    cd ~/Desktop/gdh
    ```

- Download GDH packages from [github](https://github.com/GuideDog-ETRI/gd_hri). Assume it located in ~/Desktop/gdh
    ```bash
    git clone https://github.com/GuideDog-ETRI/gd_hri .
    ```

- To use TTS of OpenAI
    ```bash
    pip install pygame
    pip install openai
    ```

- To use STT of [ReturnZero](https://www.rtzr.ai/stt)
    ```bash
    cd src/gdh_speech_audio/gdh_speech_audio
    ```
    ```bash
    # To Download definition (.proto) file
    wget https://raw.github.com/vito-ai/openapi-grpc/main/protos/vito-stt-client.proto

    # To generate gRPC code
    pip install grpcio-tools
    python -m grpc_tools.protoc -I. --python_out=. --grpc_python_out=. ./vito-stt-client.proto

    # This module requires the dependencies of grpcio and requests.
    pip install grpcio
    pip install requests
    ```
    ```bash
    cd ../../..
    ```
    Open src/gdh_speech_audio/gdh_speech_audio/vito_stt_client_pb2_grpc.py. Then change the line 6 to the following line.
    ```python
    import gdh_speech_audio.vito_stt_client_pb2 as vito__stt__client__pb2
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

- To use yolo-v8
    ```bash
    pip install ultralytics
    pip install numpy==1.25.0
    ```

- Download [Yolo models](https://drive.google.com/drive/folders/1DFF6rFE7NYYMgBKvXmN59T1wD3KD05Tb?usp=sharing) and place in the folder /models.

- Create /src/gdh_speech_audio/gdh_speech_audio/key_wallet.py file.

- Open new terminal and build
    ```bash
    ./0build.sh
    ```

# 3. 동작
## 3.1. GDH 검출기 파트
- Open three new terminals by Ctrl + Alt + T for (A) dummy photo publisher, (B) GDH node, and (C) toy test client

- (A) 이미지를 publish하기위해 (A-1), (A-2), (A-3)중에 하나를 실행한다.
- (A-1) 리코세타를 설치하지않고 녹화된 영상을 재생하겠다면, [다음 링크](https://drive.google.com/file/d/18xELEj7PeVmdU_xKqT7OH1pQPjSQ95zT/view?usp=drive_link)에서 파일을 받고 rosbag/rosbag2_2024_07_11-16_34_43 폴더로 복사 후, 다음을 실행한다. ros2 topic list 입력시 /image_raw가 있다면 성공.
    ```
    cd rosbag
    ./play.bash
    ```
- (A-2) 리코세타를 설치했고 리코세타 노드를 띄우지 않았으면 아래를 실행한다 (노드를 띄웠다면 skip).
    ```
    ros2 run theta_driver theta_driver_node
    ```

- (A-3) 특정 폴더의 이미지를 publish할 경우
    ```
    ros2 run ros2_gopro photo_pub
    ```

- (B) Run a GDH node (receiving images and processing HRI functions). 정상적이라면 이미지를 받은 알림과 GDHHeartBeat msg의 출력을 볼 수 있음.
    ```bash
    ./1run.sh
    ```

- (C) Request services
    ```bash
    . install/setup.bash

    # check image input
    ros2 topic list
    ros2 topic echo /theta/image_raw/compressed

    # start detector
    ros2 service call /GDH_start_detect gd_ifc_pkg/srv/GDHStartDetectObject "{object_types: 10}"

    # display result
    ros2 run gdh_package client_display_detect  # /GDH_detections_img

    # stop detector
    ros2 service call /GDH_stop_detect gd_ifc_pkg/srv/GDHStopDetectObject
    ```
    * Results are saved in the GDH folder.

## 3.2. STT/TTS 파트
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
 
# 4. 오류 처리
- 만약 no module py360convert가 나온다면,
    아래 명령어로 현재 수행하는 python path를 확인
    ```
    import sys
    print(sys.executable)
    ```
    알맞은 환경에 py360convert를 설치 (/usr/bin/python3가 위에서 출력한 환경이라면)
    ```
    /usr/bin/python3 -m pip install py360convert
    ```

- if gd_ifc_pkg makes new errors, install belows
    ```
    sudo apt-get update
    sudo apt-get install ros-humble-sensor-msgs 
    sudo apt-get install ros-humble-nav-msgs 
    sudo apt-get install ros-humble-grid-map-msgs ros-humble-vision-msgs
    ```

- pyaudio 설치시 에러
    ```
    sudo apt install portaudio19-dev
    pip3 install pyaudio
    ```

- 사운드 카드 미인식 혹은 설정 오류
    ```
    ALSA lib confmisc.c:855:(parse_card) cannot find card '0'
    ALSA lib conf.c:5178:(_snd_config_evaluate) function snd_func_card_inum returned error: No such file or directory
    ```
    리눅스 시스템에 로그인 필요
    아래 명령으로 사운드카드 인식상태 확인
    ```
    aplay -l
    ```

- 마이크 볼륨 컨트롤러가 안보이면,
    ```
    sudo apt install pavucontrol
    ```

<!-- - 다음과 같은 에러를 만나면,
    ```
    ALSA lib pcm_dmix.c:1032:(snd_pcm_dmix_open) unable to open slave
    ALSA lib pcm.c:2664:(snd_pcm_open_noupdate) Unknown PCM cards.pcm.rear
    ALSA lib pcm.c:2664:(snd_pcm_open_noupdate) Unknown PCM cards.pcm.center_lfe
    ALSA lib pcm.c:2664:(snd_pcm_open_noupdate) Unknown PCM cards.pcm.side
    ALSA lib pcm_oss.c:397:(_snd_pcm_oss_open) Cannot open device /dev/dsp
    ALSA lib pcm_oss.c:397:(_snd_pcm_oss_open) Cannot open device /dev/dsp
    ALSA lib confmisc.c:160:(snd_config_get_card) Invalid field card
    ALSA lib pcm_usb_stream.c:482:(_snd_pcm_usb_stream_open) Invalid card 'card'
    ALSA lib confmisc.c:160:(snd_config_get_card) Invalid field card
    ALSA lib pcm_usb_stream.c:482:(_snd_pcm_usb_stream_open) Invalid card 'card'
    ALSA lib pcm_dmix.c:1032:(snd_pcm_dmix_open) unable to open slave
    ```
    먼저 현재 재생가능한 오디오와 마이크의 card와 device 번호 획득 
    ```bash
    aplay -l
    arecord -l
    ```
    얻어진 정보로 ~/.asoundrc 파일 설정 (ex: 오디오 card 0, device 1, 마이크 card 2, device 3)
    ```
    pcm.!default {
        type hw
        card 0
        device 1
    }

    ctl.!default {
        type hw
        card 2
        device 3
    }

    ``` -->
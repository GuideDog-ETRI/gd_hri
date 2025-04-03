GDH (GuideDog HRI) module packages
==================================

# 1. 소개
[GuideDog](https://github.com/GuideDog-ETRI) 과제의 HRI파트 설치 문서이다. gdh_package (static object detection service), gdh_image (test image publisher), gdh_speech_audio (STT와 TTS), gdh_vlm_client (SA-VLM의 client 파트)로 구성되어있다.

# 2. 설치
## 2.1. gdh_packages, gdh_speech_audio
- Move to the workspace folder (assume its as ~/Desktop/gdh)
    ```bash
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

- If you have a tensorflow, we may meet un error related to tensorflow version, run below, first.
    ```
    pip uninstall tensorflow
    ```

- Check python command is runable. If not, then.
    ```
    alias python=python3
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

    Check 3 files with starting vito~ are generated in src/gdh_speech_audio/gdh_speech_audio.

    ```bash
    cd ../../..
    ```
    Open src/gdh_speech_audio/gdh_speech_audio/vito_stt_client_pb2_grpc.py. Then change the line 6 to the following line.
    ```python
    import gdh_speech_audio.vito_stt_client_pb2 as vito__stt__client__pb2
    ```

- pyaudio 설치
    ```
    sudo apt update
    sudo apt install portaudio19-dev
    pip3 install pyaudio
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

- To use yolo-v8 and rectify erp images
    ```bash
    pip install py360convert
    pip install ultralytics
    pip install numpy==1.25.0
    ```

- To use depth estimation
    ```bash
    pip install transformers
    ```

- Download Yolo models (gddemo_v2.1 version) from [Google drive](https://drive.google.com/drive/folders/1DFF6rFE7NYYMgBKvXmN59T1wD3KD05Tb?usp=sharing) or [GD Server](\\129.254.81.123\GuideDog_NAS\50_DB\gdh_src\models) and place in the folder /models.

- Create /src/gdh_speech_audio/gdh_speech_audio/key_wallet.py file.

- Orin에 설치할 경우, 다음 사항들을 추가해야 함
    - Orin에서 동작하는 버전의 라이브러리로 재설치
        ```bash
        pip uninstall tensorflow
        pip install protobuf==5.28.1 transformers==4.46.0    
        ```
    - protobuf의 msg를 못읽는다는 에러를 방지하기 위해서, src/gdh_speech_audio/gdh_speech_audio/__init__.py의 첫줄에 다음 삽입
        ```
        import os
        os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"
        ```

- Build
    ```bash
    ./0build.sh
    ```

## 2.2. gdh_image, gdh_vlm_client
- Build
    ```bash
    ./0build_vlm.sh
    ```
  
# 3. 동작
## 3.1. gdh_package (정적객체검출기) 파트
- Open three new terminals by Ctrl + Alt + T for (A) dummy photo publisher, (B) GDH node, and (C) toy test client

- (A) 이미지를 publish하기위해 (A-1), (A-2) 중에 하나를 실행한다.
- (A-1) 리코세타를 설치하지않고 녹화된 영상을 재생하겠다면, [다음 링크](https://drive.google.com/file/d/18xELEj7PeVmdU_xKqT7OH1pQPjSQ95zT/view?usp=drive_link)에서 파일을 받고 rosbag/rosbag2_2024_07_11-16_34_43 폴더로 복사 후, 다음을 실행한다. ros2 topic list 입력시 /image_raw가 있다면 성공.
    ```
    cd rosbag
    ./play.bash
    ```
- (A-2) dataset 폴더의 이미지를 publish할 경우
    ```
    ./1run_vlm.sh
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
- 먼저 SA-VLM on Orin을 설치하고, 별도 docker에서 실행 확인
- gdh_docker에서 4개의 terminal을 열고 아래 명령어를 각각 실행

    ```bash
    ./1run_vlm.sh            # 폴더에서 이미지를 주기적으로 publish하고 받고 변환하는 nodes. 화살표로 이미지 전환
    ```

    ```bash
    ./1run_vlm_recv.sh       # VLM결과 받고 TTS 서비스를 호출하는 node
    ```

    ```bash
    ./1run_vlm_toggle.sh     # 엔터를 누르면 flag on 시키고 True를 publish 노드. 아니면 False를 publish
    ```

    ```bash
    rqt_graph    # 노드간 연결 그래프와 topic을 display
    ```

위 명령어들은 아래 명령어들을 적절히 분배해서 실행함

    ```
    . install/setup.bash
    ros2 run convert_image image_publisher_node    # 폴더에서 이미지를 주기적으로 publish. 화살표로 이미지 전환
    ros2 run convert_image image_viewer_node       # 이미지 확인
    ros2 run convert_image convert_image_node      # 현재 이미지 확인.단, flag on 인경우,  Image -> Base64로 변환 후 publish
    ros2 run flag_publisher toggle_flag_node       # 엔터를 누르면 flag on 시키고 True를 publish 노드. 아니면 False를 publish
    ros2 run qwen_text_viewer qwen_console_node    # VLM결과 받는 노드
    ros2 run ai_image_subscriber image_ai_node     # VLM client 실행하는 노드
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

# 5. SA-VLM on Orin 설치
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



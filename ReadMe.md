GDH (GuideDog HRI) module packages
==================================

# 1. 소개
[GuideDog](https://github.com/GuideDog-ETRI) 과제의 HRI파트 설치 문서이다. 리코세타 설치 (옵션), GDH package 설치로 나누어져 있다.

# 2. 설치
## 2.1. 리코세타(ricoh thetaz1)

- 리코세타가 설치되었으면 skip가능
- 리코세타 설치없이 녹화된 영상을 사용하겠다면 역시 skip가능

2.1.1. 리코세타 연결
- 리코세타 카메라를 USB에 연결하고, 전원 스위치를 켠다. 모드 버튼을 눌러서 모드를 'LIVE'로 바꾼다.

2.1.2. Install driver with docker
- To share topics between hostpc and docker container, add below code in ~/.bashrc in hostpc
    ```bash
    export ROS_DOMAIN_ID=7  # max 232, 번호는 자유롭게 설정. 통신하려는 장치끼리 같은 번호 써야 함.
    export ROS_LOCALHOST_ONLY=1 # Topic will not go out of the PC. (It's ok between docker containers)
    ```

- Option A와 B 중 하나를 선택해서 설치
- (Option A) From [gd_sensor](https://github.com/GuideDog-ETRI/gd_sensor), install and run the driver and ros node for publishing images.
    ```bash
    git clone https://github.com/GuideDog-ETRI/gd_sensor
    cd gd_sensor/scripts/install_manually/thetaz1
    sudo ./install_thetaz1_driver.sh
    ```

- (Option B) From [github](https://github.com/stella-cv/theta_driver) directly, install and run the driver and ros node for publishing images. (만약 GPU가 없다면 --gpus all 라인은 지우고 할 것)
    ```bash
    git clone --recursive https://github.com/stella-cv/theta_driver
    cd theta_driver
    docker build -t theta_driver .
    docker run -it --rm \
        --privileged \
        --gpus all \
        --net=host --ipc=host\
        -e DISPLAY=$DISPLAY \
        -e ROS_DOMAIN_ID=$ROS_DOMAIN_ID \
        -e ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY} \
        -v /etc/localtime:/etc/localtime:ro \
        -v /etc/timezone:/etc/timezone:ro \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v /dev:/dev \
        theta_driver
    ros2 run theta_driver theta_driver_node
    ```
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
- /image_raw topic 을 확인
    ```bash
    ros2 topic list
    ```

## 2.2. GDH packages
- Download GDH packages from [github](https://github.com/GuideDog-ETRI/gd_hri). Assume it located in ~/Desktop/gdh
    ```bash
    git clone https://github.com/GuideDog-ETRI/gd_hri
    ```

- To use TTS of OpenAI
    ```bash
    pip install pygame
    pip install openai
    ```

- To use STT of [ReturnZero](https://www.rtzr.ai/stt)
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

- Assume all module is installed in conda env 'use_gopro'
- Open new terminal and build
    ```bash
    conda activate use_gopro
    source /opt/ros/humble/setup.bash
    cd ~/Desktop/gdh
    colcon build --packages-select gd_ifc_pkg ros2_gopro gdh_package
    ```

# 3. 동작
## 3.1. GDH 검출기 파트
- Open three new terminals by Ctrl + Alt + T for (A) dummy photo publisher, (B) GDH node, and (C) toy test client

- (A) 이미지를 publish하기위해 (A-1), (A-2), (A-3)중에 하나를 실행한다.
- (A-1) 리코세타를 설치하지않고 녹화된 영상을 재생하겠다면, [다음 링크](https://drive.google.com/file/d/18xELEj7PeVmdU_xKqT7OH1pQPjSQ95zT/view?usp=drive_link)에서 파일을 받고 rosbag/rosbag2_2024_07_11-16_34_43 폴더로 복사 후, 다음을 실행한다
    ```
    cd rosbag
    ./play.bash
    ```
- (A-2) 리코세타를 설치했고 리코세타 노드를 띄우지 않았으면 아래를 실행한다 (노드를 띄웠다면 skip).
    ```
    ros2 run theta_driver theta_driver_node
    ```
- (A-3) Run a dummy photo publisher
    ```bash
    conda activate use_gopro
    cd ~/Desktop/gdh
    . install/setup.bash
    # ros2 run ros2_gopro gopro_pub # current in error
    # ros2 run ros2_gopro webcam_pub # dummy photo publisher
    ros2 run ros2_gopro photo_pub
    ```

- (B) Run a GDH node (receiving images and processing HRI functions)
    ```bash
    conda activate use_gopro
    cd ~/Desktop/gdh
    . install/setup.bash
    ros2 run gdh_package service
    ```

- (C) Run a toy test client
    ```bash
    conda activate use_gopro
    cd ~/Desktop/gdh
    . install/setup.bash

    ros2 run ros2_gopro video_sub /photo        # check dummy photo image
    ros2 run ros2_gopro video_sub /image_raw    # check ricoh image

    ros2 run gdh_package client_det_init
    ros2 run gdh_package client_det_all
    ros2 run gdh_package client_det_term
    ```
    * Results are saved in the GDH folder.

## 3.2. STT/TTS 파트
- Open three new terminals for GDH node, toy client for sending speech codes, toy server for receiving command id.
- (A) Run GDH node
    ```bash
    conda activate use_gopro
    cd ~/Desktop/gdh
    . install/setup.bash
    ros2 run gdh_package service
    ```

- (B) Run a toy server for receiving voice command ID
    ```bash
    conda activate use_gopro
    cd ~/Desktop/gdh
    . install/setup.bash
    ros2 run gdh_package server_toy_gd_others
    ```

- (C) Run a toy client for sending speech codes
    ```bash
    conda activate use_gopro
    cd ~/Desktop/gdh
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
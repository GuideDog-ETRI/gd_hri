#!/bin/bash
echo
echo
echo "####################################################"
echo "########## Running a docker container ##########"
echo "    -  ROS2 Humble"
echo "####################################################"
echo
echo

########################################################################################################################
# Variables
########################################################################################################################

    USER_NAME="gdh"
    IMAGE_NAME="gdh"

    HOST_XAUTH_DIR=/home/"$USER"/.xauth_dir
    CONTAINER_NAME="gdh"

    # Setup X11 server authentication
    # @link http://wiki.ros.org/docker/Tutorials/GUI#The_isolated_way
    XSOCK=/tmp/.X11-unix
    XAUTH="${HOST_XAUTH_DIR}"/x11dockerize


########################################################################################################################
# Execution
########################################################################################################################
    # Setup X11 server bridge between host and container
    touch "${XAUTH}" &&
    xauth nlist "${DISPLAY}" | sed -e 's/^..../ffff/' | xauth -f "${XAUTH}" nmerge -
    chmod 644 "${XAUTH}" # not the most secure way, USE INSTEAD sublime cli

    # Run Container with X11 authentication and using same user in container and host
    # @link http://wiki.ros.org/docker/Tutorials/GUI#The_isolated_way
    #
    # Additional to the above tutorial:
    #   * I set the container --workdir in the host to persist Sublime settings and cache across restarts
    #   * I Also map my developer folder in the host to the container.
    #   * XSOCK and XAUTH only have ready access to the Host, instead of ready and write.
    sudo docker run --rm -it --gpus all --ipc=host \
        --name="${CONTAINER_NAME}" \
        --workdir=/home/gdr/workspace \
        --volume=/home/gdr/gd_workspace/gdh/workspace:/home/gdr/workspace \
        --volume="${XSOCK}":"${XSOCK}":rw \
        --volume="${XAUTH}":"${XAUTH}":rw \
        --env="XAUTHORITY=${XAUTH}" \
        --env="DISPLAY" \
        --env="QT_X11_NO_MITSHM=1" \
        --env="NVIDIA_VISIBLE_DEVICES=all" \
        --env="NVIDIA_DRIVER_CAPABILITIES=all" \
        --net=host \
        --privileged \
        --device=/dev/snd:/dev/snd \
        -e PULSE_SERVER=unix:${XDG_RUNTIME_DIR}/pulse/native \
        -v ${XDG_RUNTIME_DIR}/pulse/native:${XDG_RUNTIME_DIR}/pulse/native \
        -v ~/.config/pulse/cookie:/root/.config/pulse/cookie \
        -v /dev:/dev \
        -v /dev/bus/usb:/dev/bus/usb \
        "${IMAGE_NAME}" bash
        #--network=gd_network \

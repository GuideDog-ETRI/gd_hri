
####################################################
#
#    Docfile :
#             - ubuntu 22.04 
#             - ros2 Humble
####################################################


FROM osrf/ros:humble-desktop-full

ENV NO_AT_BRIDGE=1
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get upgrade -y && apt-get install apt-utils -y \
    && apt-get install -y --no-install-recommends \
    inetutils-ping \
    lsb-release \
    net-tools \
    unzip \
    vim \
    zip \
    curl \
    git \
    wget \
    ca-certificates \
    sudo \    
    bzip2 \
    libx11-6 \
    software-properties-common \   
    && apt-get autoclean -y \
    && apt-get autoremove -y \
    && rm -rf /var/lib/apt/lists/*


RUN apt-get update \
    && apt-get install -y --no-install-recommends \
    dbus-x11 \
    xvfb \
    x11vnc \
    x11-xserver-utils \
    at-spi2-core \
    python3 \
    python3-pip \
    python-is-python3 \    
    && apt-get purge -y pm-utils xscreensave* \
    && apt-get autoclean -y \
    && apt-get autoremove -y \
    && rm -rf /var/lib/apt/lists/*
    
    
#supress setrlimit(RLIMIT_CORE) error
RUN \
    echo "Set disable_coredump false" >> /etc/sudo.conf

# Create a non-root user and switch to it
RUN adduser --disabled-password --gecos '' --shell /bin/bash gdh 
RUN echo "gdh ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/90-gdh
USER gdh

# All users can use /home/user as their home directory
ENV HOME=/home/gdh
#RUN chmod 777 /home/gdh


ENV \
    USER=gdh \
    HOME=/home/gdh


RUN rosdep fix-permissions \
    && rosdep update \
    && echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
#COPY app.desktop /home/user/


RUN mkdir -p /home/gdh/workspace
WORKDIR /home/gdh/workspace
RUN chown -R gdh /home/gdh/workspace
RUN chgrp -R gdh /home/gdh/workspace

ENV SHELL /bin/bash


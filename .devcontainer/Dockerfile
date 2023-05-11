FROM ros:noetic-ros-base

RUN apt-get update && \
    apt upgrade -y && \
# tzdata
    ln -fs /usr/share/zoneinfo/Europe/Warsaw /etc/localtime && \
    apt-get install -y tzdata && \
    dpkg-reconfigure --frontend noninteractive tzdata

# Install dependencies
RUN apt-get update && \
    apt install -y \
 dialog apt-utils curl gnupg lsb-release \
 vim htop tree less locate ninja-build wget \
 ffmpeg build-essential cmake git unzip &&\
    rm -rf /var/lib/apt/lists/*

RUN apt update &&\
    apt install -y \
 x11-apps \
 python3-pip \
 clangd-12 &&\
    pip install serial pyserial pyproj

ARG MASTER_URI
ARG ROS_IP

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc &&\
    echo "export ROS_MASTER_URI=${MASTER_URI}" >> ~/.bashrc &&\
    echo "export ROS_IP=${ROS_IP}" >> ~/.bashrc &&\
    echo "alias runTachimetr=\"sudo rfcomm bind rfcomm0 00:12:F3:43:5C:F7\"" >> ~/.bash_aliases

# Set up auto-source of workspace for root user
ARG WORKSPACE

RUN mkdir -p ${WORKSPACE}

WORKDIR ${WORKSPACE}

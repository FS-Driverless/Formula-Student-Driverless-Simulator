#Using nvidia vulkan container as base
ARG BASE_IMAGE=nvidia/vulkan:1.1.121-cuda-10.1-beta.1-ubuntu18.04
FROM $BASE_IMAGE
RUN apt-get update
RUN apt-get install \
	python3 \
	python3-pip \
	sudo \
	libglu1-mesa-dev \
	xdg-user-dirs \
	pulseaudio \
	sudo \
	vulkan-utils \
	wget \
	libvulkan1 \
	mesa-vulkan-drivers \
	x11-xserver-utils \
	-y --no-install-recommends
RUN pip3 install --upgrade pip
RUN pip3 install setuptools wheel

# Enable Vulkan support
ARG VULKAN_VERSION="1.2.148"
ENV NVIDIA_DRIVER_CAPABILITIES compute,graphics,utility,display
RUN export $(cat /etc/os-release | grep --color=never 'UBUNTU_CODENAME') && \
    wget -qO - https://packages.lunarg.com/lunarg-signing-key-pub.asc | apt-key add - && \
    wget -qO /etc/apt/sources.list.d/lunarg-vulkan-${VULKAN_VERSION}-${UBUNTU_CODENAME}.list \
    https://packages.lunarg.com/vulkan/${VULKAN_VERSION}/lunarg-vulkan-${VULKAN_VERSION}-${UBUNTU_CODENAME}.list && \
    apt-get update && apt-get install -y --no-install-recommends vulkan-sdk && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*
RUN VULKAN_API_VERSION=`dpkg -s libvulkan1 | grep -oP 'Version: [0-9|\.]+' | grep -oP '[0-9|\.]+'` && \
    mkdir -p /etc/vulkan/icd.d/ && \
    echo \
    "{\
        \"file_format_version\" : \"1.0.0\",\
        \"ICD\": {\
            \"library_path\": \"libGLX_nvidia.so.0\",\
            \"api_version\" : \"${VULKAN_API_VERSION}\"\
        }\
    }" > /etc/vulkan/icd.d/nvidia_icd.json

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*

# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros1-latest.list

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ENV ROS_DISTRO melodic

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-melodic-desktop-full=1.4.1-0* \
    && rm -rf /var/lib/apt/lists/*

RUN adduser --force-badname --disabled-password --gecos '' --shell /bin/bash fsdssim_user && \ 
	echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers && \ 
	adduser fsdssim_user sudo && \ 
	adduser fsdssim_user audio && \ 
	adduser fsdssim_user video

USER fsdssim_user
WORKDIR /home/fsdssim_user

# Copy Simulator settings.json to Formula-Student-Driverless-Simulator Hard code from Simulator
RUN mkdir -p /home/fsdssim_user/Formula-Student-Driverless-Simulator/
COPY settings.json /home/fsdssim_user/Formula-Student-Driverless-Simulator/


RUN sudo chown -R fsdssim_user /home/fsdssim_user


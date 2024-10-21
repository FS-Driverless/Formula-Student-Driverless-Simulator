FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    git \
    build-essential \
    cmake \
    libboost-all-dev \
    libasio-dev \
    libtinyxml2-dev \
    libcurl4-openssl-dev \
    libglfw3-dev \
    libglew-dev \
    libpng-dev \
    libjpeg-dev \
    libopencv-dev \
    libtiff-dev \
    python3 \
    python3-pip \
    python3-dev \
    unzip \
    wget \
    lsb-release \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    clang-8 \
    lld-8 \
    llvm-8 \
    llvm-8-dev \
    llvm-8-tools

RUN apt-get update && apt-get install -y \
    g++ \
    libc++-dev \
    libc++abi-dev

RUN apt-get update && apt-get install -y \
    wget \
    software-properties-common \
    gnupg \
    rsync \
    && apt-get clean

WORKDIR /home/airsim/Formula-Student-Driverless-Simulator/AirSim

ENTRYPOINT ["/bin/bash"]


FROM ghcr.io/sloretz/ros:jazzy-simulation

ENV ROS_DISTRO="jazzy" 
ENV DEBIAN_FRONTEND="noninteractive"

RUN apt update && \
    apt install -y \
        gdb \
        ninja-build && \
    rm -rf /var/lib/apt/lists/*

COPY . /ros2_ws/src/WheeledBipedSim

WORKDIR /ros2_ws

RUN apt update && \
    rosdep update && \
    rosdep install --from-paths src -r -y --ignore-src && \
    rm -rf /var/lib/apt/lists/*

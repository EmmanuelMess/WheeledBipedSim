FROM ghcr.io/sloretz/ros:jazzy-simulation

ENV ROS_DISTRO="jazzy" 

RUN apt update && \
    apt install -y \
        ros-${ROS_DISTRO}-controllers && \
    rm -rf /var/lib/apt/lists/*

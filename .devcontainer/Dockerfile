# Simple Dockerfile that uses the existing LCAS image

FROM lcas.lincoln.ac.uk/devcontainer/ros2-teaching:4 AS base


USER root
ENV DEBIAN_FRONTEND=noninteractive

# Add any additional customizations here if needed
# For example, install additional packages:FROM lcas.lincoln.ac.uk/devcontainer/ros2-teaching:4FROM nvidia/cuda:11.8-devel-ubuntu22.04
RUN apt-get update && apt-get install -y \
    htop \
    && \
    rm -rf /var/lib/apt/lists/*


FROM base AS deps_installed
# Switch to ros user
USER ros
# copy all package.xml from the src tree and use rosdep to install all dependencies, 
RUN --mount=type=bind,source=./src,target=/ros_ws/src \
    . /opt/ros/lcas/install/setup.sh && \
    sudo apt-get update && \
    rosdep update --rosdistro ${ROS_DISTRO} && \
    rosdep install -y --from-paths /ros_ws/src --ignore-src -r --rosdistro ${ROS_DISTRO} && \
    sudo rm -rf /var/lib/apt/lists/*

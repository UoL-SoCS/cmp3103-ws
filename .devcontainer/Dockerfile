# Simple Dockerfile that uses the existing LCAS image

FROM lcas.lincoln.ac.uk/devcontainer/ros2-teaching:4


USER root
ENV DEBIAN_FRONTEND=noninteractive

# Add any additional customizations here if needed
# For example, install additional packages:FROM lcas.lincoln.ac.uk/devcontainer/ros2-teaching:4FROM nvidia/cuda:11.8-devel-ubuntu22.04
# RUN apt-get update && apt-get install -y \
#     your-package-here \
#     && rm -rf /var/lib/apt/lists/*


# Switch to ros user
USER ros

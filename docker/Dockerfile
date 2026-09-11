# syntax=docker/dockerfile:1
FROM nvcr.io/nvidia/isaac-sim:5.1.0 AS isaac
FROM ros:jazzy-ros-base-noble
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
    libatomic1 libegl1 libgl1 libglib2.0-0 libgomp1 libice6 libsm6 \
    libx11-6 libxext6 libxi6 libxrandr2 libxrender1 libxt6 libxtst6 \
    libxkbcommon0 libxkbcommon-x11-0 libxcb-cursor0 libnss3 libasound2t64 \
    libvulkan1 libglu1-mesa libfreetype6 libfontconfig1 libdbus-1-3 \
    python3-numpy python3-colcon-common-extensions ros-jazzy-rviz2 \
    ros-jazzy-rmw-fastrtps-cpp zsh ffmpeg xauth \
    && rm -rf /var/lib/apt/lists/*
COPY --from=isaac --chown=1000:1000 /isaac-sim /isaac-sim
ENV ISAAC_SIM_PATH=/isaac-sim ROS_DOMAIN_ID=73 \
    NVIDIA_VISIBLE_DEVICES=all NVIDIA_DRIVER_CAPABILITIES=all
WORKDIR /workspace
COPY --chown=1000:1000 . /workspace
RUN source /opt/ros/jazzy/setup.bash && colcon build --base-paths src --packages-select aprl_robot_sim nero_description --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 \
    && mkdir -p /workspace/.runtime && chown 1000:1000 /workspace/.runtime
ENTRYPOINT []
CMD ["./scripts/sim", "--headless", "--scene", "office"]

FROM ros:humble

ARG DEBIAN_FRONTEND=noninteractive

# Install both RMW implementations + diagnostics
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-rmw-fastrtps-cpp \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-demo-nodes-cpp \
    ros-humble-rmw-dds-common \
    tcpdump \
    iproute2 \
    iputils-ping \
    tshark \
    && rm -rf /var/lib/apt/lists/*

# Pre-source ROS in bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

WORKDIR /workspace
COPY scripts/ /workspace/scripts/
RUN chmod +x /workspace/scripts/*.sh 2>/dev/null || true

CMD ["bash"]

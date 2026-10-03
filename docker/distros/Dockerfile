# ROS 2 test environment for RaftROS, one image per distribution:
#   docker build --build-arg ROS_DISTRO=kilted -t raftros-ros:kilted docker/distros
# Run with --network host so the router, DDS discovery and the tests share the
# host's LAN address - the board connects to it exactly as to a native install.
ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base
ARG ROS_DISTRO
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-rmw-zenoh-cpp \
        ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
        python3-venv python3-pip bc procps curl \
    && rm -rf /var/lib/apt/lists/*

# zenoh-python for raw queries and liveliness probes (not ROS) - its own venv
RUN python3 -m venv /opt/zenoh-venv && \
    /opt/zenoh-venv/bin/pip install --no-cache-dir "eclipse-zenoh>=1.4,<2"

ENV ROS_DISTRO=${ROS_DISTRO}
COPY tests/ /opt/raftros-tests/
RUN chmod +x /opt/raftros-tests/*.sh
WORKDIR /opt/raftros-tests
CMD ["sleep", "infinity"]

# ROS 2 Humble
FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

# Install system and ROS development tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    wget \
    curl \
    build-essential \
    python3-pip \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-rosdep \
    ros-dev-tools \
    ros-humble-diagnostic-aggregator \
    ros-humble-rqt-robot-monitor \
    python3-pykdl \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# Create non-root user
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && usermod -aG dialout,plugdev $USERNAME

# Create ROS workspace
WORKDIR /home/$USERNAME/ros2_ws

# Copy team packages and dependency manifest
COPY ros2_ws/src ./src
COPY dependencies.repos /tmp/dependencies.repos

# Import pinned DOBOT driver
RUN vcs import src < /tmp/dependencies.repos

# Install DOBOT Python requirements
RUN python3 -m pip install --no-cache-dir \
    -r src/magician_ros2/requirements.txt

# Install ROS package dependencies
RUN apt-get update \
    && rosdep update \
    && rosdep install --from-paths src \
        --ignore-src --rosdistro humble -y \
    && rm -rf /var/lib/apt/lists/*

# Build complete ROS workspace
RUN /bin/bash -c \
    "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

# Give workspace ownership to non-root user
RUN chown -R $USERNAME:$USERNAME /home/$USERNAME/ros2_ws

USER $USERNAME
WORKDIR /home/$USERNAME/ros2_ws

# Source ROS and the built workspace in interactive Bash shells
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc \
    && echo "source /home/$USERNAME/ros2_ws/install/setup.bash" >> ~/.bashrc

CMD ["bash"]

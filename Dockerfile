# Offizielles ROS Image
FROM osrf/ros:humble-desktop-full

# Keine Prompts bei Installation
ENV DEBIAN_FRONTEND=noninteractive

# Utilities, git, and python tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    wget \
    curl \
    python3-pip \
    python3-colcon-common-extensions \
    ros-dev-tools \
    ros-humble-diagnostic-aggregator \
    ros-humble-rqt-robot-monitor \
    python3-pykdl \
    && rm -rf /var/lib/apt/lists/*

# Non-root user
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# User zu 'dialout' und 'plugdev' Gruppen hinzufügen 
# Für Zugriff zu USB
RUN usermod -aG dialout,plugdev $USERNAME

# Switch zu User
USER $USERNAME
WORKDIR /home/$USERNAME

# Automatically source ROS 2 in every bash shell for user
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

CMD ["bash"]
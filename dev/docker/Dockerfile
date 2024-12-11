FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

# Install essential tools
RUN apt-get update && apt-get install -y \
    build-essential \
    curl \
    git \
    python3-pip \
    wget \
    lsb-release \
    gnupg2 \
    cmake \
    && rm -rf /var/lib/apt/lists/*

# Install dependencies
RUN apt-get update && apt-get install -y \
    libeigen3-dev \
    libopencv-dev \
    libboost-all-dev \
    libtbb-dev \
    && rm -rf /var/lib/apt/lists/*

# Install GTSAM from source
RUN git clone https://github.com/borglab/gtsam.git /tmp/gtsam \
    && cd /tmp/gtsam \
    && mkdir build && cd build \
    && cmake .. -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_WITH_TBB=ON \
    && make -j$(nproc) \
    && make install \
    && cd / && rm -rf /tmp/gtsam

# Install ROS2 Humble
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt-get update \
    && apt-get install -y \
        software-properties-common \
    && add-apt-repository universe \
    && apt-get update && apt-get install -y \
        ros-humble-ros-base \
        ros-humble-cv-bridge \
        ros-humble-message-filters \
        ros-humble-libstatistics-collector \
        python3-colcon-common-extensions \
        python3-rosdep \
    && rosdep init \
    && rosdep update \
    && rm -rf /var/lib/apt/lists/*

# Create non-root user
ARG USERNAME=developer
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME

USER $USERNAME
WORKDIR /workspace

# Setup ROS2 environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
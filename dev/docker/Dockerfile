FROM nvidia/cuda:11.8.0-devel-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive

# Install essential tools
RUN apt-get update && apt-get install -y \
    build-essential \
    curl \
    git \
    wget \
    lsb-release \
    gnupg2 \
    cmake \
    clang \
    lld \
    software-properties-common \
    && add-apt-repository ppa:deadsnakes/ppa \
    && apt-get update \
    && apt-get install -y \
    python3.10 \
    python3.10-venv \
    python3.10-dev \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install dependencies
RUN apt-get update && apt-get install -y \
    wget \
    lsb-release \
    gnupg2 \
    && apt-get update \
    && apt-get install -y \
    libeigen3-dev \
    libopencv-dev \
    libboost-all-dev \
    libtbb-dev \
    && rm -rf /var/lib/apt/lists/*


# Install GTSAM from source
RUN git clone https://github.com/borglab/gtsam.git /tmp/gtsam \
    && cd /tmp/gtsam \
    && git checkout 4.2.0 \
    && mkdir build \
    && cd build \
    && cmake .. -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_WITH_TBB=ON -DCMAKE_CXX_FLAGS="-Wno-error=return-type" \
    && make -j$(nproc) \
    && make install \
    && cd / \
    && rm -rf /tmp/gtsam

# # Install ROS2 Humble
# RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
#     && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
#     && apt-get update \
#     && apt-get install -y \
#         software-properties-common \
#     && add-apt-repository universe \
#     && apt-get update && apt-get install -y \
#         ros-humble-ros-base \
#         ros-humble-cv-bridge \
#         ros-humble-message-filters \
#         ros-humble-libstatistics-collector \
#         python3-colcon-common-extensions \
#         python3-rosdep \
#     && rosdep init \
#     && rosdep update \
#     && rm -rf /var/lib/apt/lists/*
# Install Apache Arrow
RUN apt-get update && apt-get install -y \
    ca-certificates \
    lsb-release \
    wget \
    && wget https://apache.jfrog.io/artifactory/arrow/$(lsb_release --id --short | tr 'A-Z' 'a-z')/apache-arrow-apt-source-latest-$(lsb_release --codename --short).deb \
    && apt-get install -y ./apache-arrow-apt-source-latest-$(lsb_release --codename --short).deb \
    && apt-get update \
    && apt-get install -y \
    libarrow-dev \
    libarrow-glib-dev \
    libarrow-dataset-dev \
    libarrow-dataset-glib-dev \
    libarrow-acero-dev \
    libarrow-flight-dev \
    libarrow-flight-glib-dev \
    libarrow-flight-sql-dev \
    libarrow-flight-sql-glib-dev \
    && rm -rf /var/lib/apt/lists/* \
    && rm apache-arrow-apt-source-latest-$(lsb_release --codename --short).deb

# Install X11 and GUI-related dependencies
RUN apt-get update && apt-get install -y \
    libxkbcommon-x11-0 \
    libxkbcommon-dev \
    libx11-dev \
    libxcb1-dev \
    libxcb-render0-dev \
    libxcb-shape0-dev \
    libxcb-xfixes0-dev \
    libxcursor-dev \
    libxrandr-dev \
    libxi-dev \
    libgl1-mesa-dev \
    libegl1-mesa \
    libegl1-mesa-dev \
    libgl1-mesa-dri \
    libgl1-mesa-glx \
    libgles2-mesa-dev \
    && rm -rf /var/lib/apt/lists/*

# Install Vulkan dependencies
RUN apt-get update && apt-get install -y \
    vulkan-tools \
    libvulkan1 \
    libvulkan-dev \
    && rm -rf /var/lib/apt/lists/*

# Add this after installing the Vulkan packages
RUN mkdir -p /usr/share/vulkan/icd.d/ && \
    echo '{ \
        "file_format_version": "1.0.0", \
        "ICD": { \
            "library_path": "libGLX_nvidia.so.0", \
            "api_version": "1.2.0" \
        } \
    }' > /usr/share/vulkan/icd.d/nvidia_icd.json

# Install zsh and other required packages earlier in the Dockerfile
RUN apt-get update && apt-get install -y \
    zsh \
    git-lfs \
    htop \
    tmux \
    vim \
    curl \
    fonts-powerline \
    && rm -rf /var/lib/apt/lists/*

# Create non-root user
ARG USERNAME=developer
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chsh -s $(which zsh) $USERNAME

USER $USERNAME
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | bash -s -- -y
# Source the cargo environment in the same RUN command
RUN . $HOME/.cargo/env && \
    cargo install cargo-binstall && \
    cargo binstall rerun-cli

WORKDIR /workspace

# Setup ROS2 environment
# RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Copy setup script into the container
COPY --chmod=755 dev/docker/setup.sh /tmp/setup.sh

# Run setup script for the developer user
RUN /tmp/setup.sh

ENTRYPOINT ["/bin/zsh"]

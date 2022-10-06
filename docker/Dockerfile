FROM arm64v8/ubuntu as base

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt update && \
    apt install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*

# install packages
RUN apt update && apt install -q -y --no-install-recommends \
    curl \
    git \
    bash-completion \
    dirmngr \
    gnupg2 \
    lsb-release \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# setup locale
RUN apt update && apt install -y locales 
ENV LANG en_US.UTF-8
ENV LC_ALL en_US.UTF-8

# Setup resources
RUN apt-cache policy | grep universe && \
    apt update && apt install curl gnupg lsb-release && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu jammy main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt update && apt upgrade -y

# Install desktop version for ROS2 Humble
RUN DEBIAN_FRONTEND=noninteractive apt install -y ros-humble-desktop 

# Resolve dependencies and install colcon tool
WORKDIR /root/dev_ws/src
RUN git clone https://github.com/ros/ros_tutorials.git -b humble-devel
WORKDIR /root/dev_ws

RUN apt-get install python3-rosdep -y && \
    rosdep init && \
    rosdep update && \
    rosdep install -i --from-path src --rosdistro humble -y && \
    apt install python3-colcon-common-extensions -y
WORKDIR /

# Setup aliases
RUN echo "alias source_humble='source /opt/ros/humble/setup.bash'" >> /root/.bashrc

# Setup VNC

CMD ["bash"]

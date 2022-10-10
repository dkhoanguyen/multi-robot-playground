FROM arm64v8/ubuntu:22.04 as base

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
RUN DEBIAN_FRONTEND=noninteractive apt install -y ros-humble-desktop-full

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

# Install ros2 bridge server
RUN apt install -y ros-humble-rosbridge-server
# Setup aliases
RUN echo "alias source_humble='source /opt/ros/humble/setup.bash'" >> /root/.bashrc
EXPOSE 9090

# Setup dependencies for build gazebo from source
RUN apt update && \
    apt install -y libignition-msgs5-dev \
    libignition-transport8-dev \
    libignition-common3-dev \
    libignition-fuel-tools4-dev \
    libqwt-qt5-dev \
    libtar-dev \
    libfreeimage-dev

RUN mkdir /gazebo

# Clone and install sdf9
RUN cd /gazebo && \
    git clone https://github.com/gazebosim/sdformat && \
    cd sdformat && \
    git checkout sdf9 &&\
    mkdir build && \
    cd build && \
    cmake .. &&\
    make install

# Clone and install gazebo
RUN cd /gazebo && \
    git clone https://github.com/osrf/gazebo && \
    cd gazebo && \
    mkdir build && \
    cd build && \
    cmake .. &&\
    make install

# Gzserver
EXPOSE 11345

#Gzweb
EXPOSE 8080

# Rosbridge
EXPOSE 9090

# Install gzweb
RUN apt update && \
    apt install -y nodejs

SHELL ["/bin/bash", "-c"]
RUN cd /gazebo && \
    git clone https://github.com/osrf/gzweb && \
    cd gzweb && \
    git checkout gzweb_1.4.1 && \
    source /usr/local/share/gazebo-11/setup.bash && \  
    export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib


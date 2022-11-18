FROM arm64v8/ubuntu:20.04 as base

# Install system build dependencies
ENV PATH=/usr/local/bin:$PATH
RUN apt-get update -qq \
&& DEBIAN_FRONTEND=noninteractive apt-get install -yq git wget libssl-dev build-essential \
 ninja-build python3 pkgconf libglib2.0-dev \
&& apt-get clean \
&& rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# Install CMake 3.21.1
RUN wget -q "https://cmake.org/files/v3.21/cmake-3.21.0-rc1-linux-aarch64.sh" \
&& chmod a+x cmake-3.21.0-rc1-linux-aarch64.sh \
&& ./cmake-3.21.0-rc1-linux-aarch64.sh --prefix=/usr/local/ --skip-license \
&& rm cmake-3.21.0-rc1-linux-aarch64.sh

WORKDIR /home/
RUN git clone https://github.com/google/or-tools.git

ENV PROJECT=or-tools
ENV TARGET=aarch64-linux-gnu
RUN ./or-tools/tools/cross_compile.sh build

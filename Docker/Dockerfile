# --- Dockerfile for OR-Tools 9.14 + SCIP 9.2.4 + I-Drone project ---

FROM ubuntu:22.04

# --- Variables ---
ENV DEBIAN_FRONTEND=noninteractive
ENV SCIP_VERSION=v924
ENV ORTOOLS_VERSION=v9.14

# --- Install basic dependencies, Boost, Python3, Meson, Ninja ---
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    git \
    wget \
    unzip \
    curl \
    software-properties-common \
    zlib1g-dev \
    libbz2-dev \
    libprotobuf-dev \
    protobuf-compiler \
    pkg-config \
    python3-pip \
    ninja-build \
    libboost-all-dev \
    && pip3 install --upgrade meson \
    && rm -rf /var/lib/apt/lists/*

# --- Install modern CMake (3.27) ---
RUN wget https://github.com/Kitware/CMake/releases/download/v3.27.7/cmake-3.27.7-linux-x86_64.sh -O /tmp/cmake.sh && \
    sh /tmp/cmake.sh --skip-license --prefix=/usr/local && \
    rm /tmp/cmake.sh

# --- Install SCIP 9.2.4 ---
RUN git clone https://github.com/scipopt/scip.git /tmp/scip && \
    cd /tmp/scip && \
    git checkout ${SCIP_VERSION} && \
    git submodule update --init --recursive && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release \
             -DBUILD_SHARED_LIBS=ON \
             -DSCIP_ENABLE_PAPILO=OFF \
             -DSCIP_BUILD_TESTS=OFF \
             -DAUTOBUILD=ON \
             -DCMAKE_INSTALL_PREFIX=/usr/local \
             -DSCIP_INSTALL=ON && \
    make -j$(nproc) && make install && \
    rm -rf /tmp/scip

# --- Install OR-Tools 9.14 (C++17) ---
RUN wget https://github.com/google/or-tools/archive/refs/tags/${ORTOOLS_VERSION}.tar.gz -O /tmp/ortools.tar.gz && \
    tar -xzf /tmp/ortools.tar.gz -C /tmp && \
    cd /tmp/or-tools-9.14 && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release \
             -DBUILD_DEPS=ON \
             -DBUILD_PYTHON=OFF \
             -DSCIP_ROOT=/usr/local \
             -DCMAKE_CXX_STANDARD=17 && \
    make -j$(nproc) && make install && \
    rm -rf /tmp/or-tools-9.14 /tmp/ortools.tar.gz

# --- Install Signal Server and build only the binary ---
WORKDIR /opt/signal_server
RUN git clone https://github.com/valderez/Signal-Server && cd Signal-Server && \
    g++ -std=c++17 -O3 -pthread main.cpp cost.cpp fspl.cpp hata.cpp itm.cpp -o /opt/signal_server/signal-server && \
    cd /opt/signal_server && rm -rf Signal-Server

# --- Copy I-Drone source code and build ---
WORKDIR /app
COPY . /app

RUN meson setup build . && \
    meson configure build -Dcpp_std=c++17 && \
    meson compile -C build && \
    mkdir -p /opt/I-Drone && \
    cp build/Planner/Planner /opt/I-Drone/ && \
    cp build/PLD/PLD /opt/I-Drone/ && \
    rm -rf /app

# --- Set working directory and default command ---
WORKDIR /opt/I-Drone
CMD ["bash"]

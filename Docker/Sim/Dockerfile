# --- Unified Dockerfile: I-Drone build + PX4 Simulation environment ---
# Base: Ubuntu 22.04 for compatibility with both I-Drone dependencies and PX4

FROM ubuntu:22.04

# --- Environment Variables ---
ENV DEBIAN_FRONTEND=noninteractive
ENV DISPLAY=:0
ENV QT_X11_NO_MITSHM=1

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

# --- Install Gazebo Classic and PX4/MAVSDK dependencies ---
RUN apt-get update && apt-get install -y --no-install-recommends \
    lsb-release \
    gnupg \
    && wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && apt-get update && apt-get install -y --no-install-recommends \
    gazebo \
    libgazebo-dev \
    python3 \
    python3-pip \
    python3-lxml \
    libopencv-dev \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    libasio-dev \
    libtinyxml2-dev \
    libcurl4-openssl-dev \
    && rm -rf /var/lib/apt/lists/*

# --- Install Python packages for PX4 ---
RUN pip3 install --no-cache-dir \
    kconfiglib \
    jinja2 \
    jsonschema \
    future \
    toml \
    pyros-genmsg \
    packaging \
    numpy \
    empy==3.3.4 \
    pyyaml

# --- Clone PX4 Autopilot ---
WORKDIR /root/tfg
RUN git clone https://github.com/PX4/PX4-Autopilot.git /root/tfg/PX4-Autopilot && \
    cd /root/tfg/PX4-Autopilot && \
    git submodule update --init --recursive && \
    git config --global --add safe.directory /root/tfg/PX4-Autopilot

# --- MAVSDK ---
# Clone MAVSDK
RUN git clone https://github.com/mavlink/MAVSDK.git /root/MAVSDK && \
    cd /root/MAVSDK && \
    git submodule update --init --recursive

# Compile and install MAVSDK (C++ library + CLI)
RUN cmake -B /root/MAVSDK/build -S /root/MAVSDK -DCMAKE_BUILD_TYPE=Release && \
    cmake --build /root/MAVSDK/build --target install -j$(nproc) && \
    ldconfig

# --- Configure library paths ---
RUN echo "/usr/local/lib" >> /etc/ld.so.conf.d/local.conf && \
    ldconfig

# --- Copy I-Drone source code and build only Drone and libs ---
WORKDIR /app
COPY Drone /app/Drone
COPY libs /app/libs
COPY meson_drone.build /app/meson.build

# Regenerate protobuf files with system protobuf (3.x)
RUN cd /app/libs/proto && \
    protoc --experimental_allow_proto3_optional --cpp_out=../generated_proto messages_algo.proto && \
    protoc --experimental_allow_proto3_optional --cpp_out=../generated_proto messages_drone.proto && \
    protoc --experimental_allow_proto3_optional --cpp_out=../generated_proto messages_pld.proto

# Build Drone module
RUN meson setup build . && \
    meson compile -C build && \
    mkdir -p /opt/I-Drone && \
    cp build/Drone/Drone /opt/I-Drone/ && \
    rm -rf /app

# --- Final working directory ---
WORKDIR /opt/I-Drone

# Keep container open
CMD ["bash"]

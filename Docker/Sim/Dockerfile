FROM px4io/px4-dev-simulation-focal:latest

# Entorno X11
ENV DISPLAY=:0
ENV QT_X11_NO_MITSHM=1

# Carpeta de trabajo
WORKDIR /root/tfg

# Clonar PX4 y submodules
RUN git clone https://github.com/PX4/PX4-Autopilot.git /root/tfg/PX4-Autopilot && \
    git -C /root/tfg/PX4-Autopilot submodule update --init --recursive

# Marcar safe.directory para Git
RUN git config --global --add safe.directory /root/tfg/PX4-Autopilot

# --- Actualizar CMake a >= 3.22 ---
RUN apt-get update && apt-get install -y software-properties-common && \
    apt-add-repository ppa:git-core/ppa && \
    apt-get update && \
    apt-get install -y wget build-essential libssl-dev && \
    wget -q https://cmake.org/files/v3.26/cmake-3.26.4-linux-x86_64.sh && \
    chmod +x cmake-3.26.4-linux-x86_64.sh && \
    ./cmake-3.26.4-linux-x86_64.sh --skip-license --prefix=/usr/local && \
    rm cmake-3.26.4-linux-x86_64.sh

# --- MAVSDK ---
# Instala dependencias necesarias para compilar MAVSDK
RUN apt-get update && apt-get install -y \
    git build-essential cmake libasio-dev libtinyxml2-dev libcurl4-openssl-dev \
    && rm -rf /var/lib/apt/lists/*

# Clona MAVSDK en /root/MAVSDK
RUN git clone https://github.com/mavlink/MAVSDK.git /root/MAVSDK && \
    git -C /root/MAVSDK submodule update --init --recursive

# Compila e instala MAVSDK (C++ library + CLI)
RUN cmake -B /root/MAVSDK/build -S /root/MAVSDK -DCMAKE_BUILD_TYPE=Release && \
    cmake --build /root/MAVSDK/build --target install -j$(nproc)

# Cambiar WORKDIR final al repositorio clonado
WORKDIR /root/tfg/PX4-Autopilot

# Mantener el contenedor abierto
CMD ["bash"]

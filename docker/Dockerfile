FROM osrf/ros:noetic-desktop-full

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ARG DEBIAN_FRONTEND=noninteractive
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000

ENV DEBIAN_FRONTEND=${DEBIAN_FRONTEND} \
    NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=all \
    QT_X11_NO_MITSHM=1 \
    __GLX_VENDOR_LIBRARY_NAME=nvidia \
    COIN_LIO_AUTO_BUILD=1 \
    FOXGLOVE_BRIDGE_PORT=8765 \
    CATKIN_WS=/home/${USERNAME}/catkin_ws \
    HOME=/home/${USERNAME}

# RViz/OpenGL uses the host NVIDIA driver runtime when available.
RUN apt-get update && apt-get install -y --no-install-recommends \
    bash-completion \
    build-essential \
    ca-certificates \
    clangd \
    cmake \
    curl \
    dbus-x11 \
    git \
    libboost-all-dev \
    libegl1 \
    libeigen3-dev \
    libgl1-mesa-glx \
    libglvnd0 \
    libglx0 \
    libgoogle-glog-dev \
    libopencv-dev \
    libpcl-dev \
    mesa-utils \
    pkg-config \
    python3-catkin-tools \
    python3-vcstool \
    sudo \
    wget \
    x11-apps \
    xauth \
    ros-noetic-cmake-modules \
    ros-noetic-cv-bridge \
    ros-noetic-eigen-conversions \
    ros-noetic-foxglove-bridge \
    ros-noetic-pcl-ros \
    ros-noetic-tf \
    && rm -rf /var/lib/apt/lists/*

RUN set -eux; \
    if ! getent group "${USER_GID}" >/dev/null; then \
        groupadd --gid "${USER_GID}" "${USERNAME}"; \
    fi; \
    if ! id -u "${USERNAME}" >/dev/null 2>&1; then \
        useradd --uid "${USER_UID}" --gid "${USER_GID}" --create-home --shell /bin/bash "${USERNAME}"; \
    fi; \
    usermod -aG sudo,dialout,video "${USERNAME}"; \
    if getent group render >/dev/null; then usermod -aG render "${USERNAME}"; fi; \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > "/etc/sudoers.d/${USERNAME}"; \
    chmod 0440 "/etc/sudoers.d/${USERNAME}"; \
    mkdir -p "${CATKIN_WS}/src" "${HOME}/Desktop"; \
    chown -R "${USER_UID}:${USER_GID}" "${HOME}"

COPY docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

USER ${USERNAME}
WORKDIR ${CATKIN_WS}

RUN source /opt/ros/noetic/setup.bash \
    && catkin_make -DCMAKE_BUILD_TYPE=Release -j"$(nproc)" \
    && echo "source /opt/ros/noetic/setup.bash" >> "${HOME}/.bashrc" \
    && echo "if [ -f ${CATKIN_WS}/devel/setup.bash ]; then source ${CATKIN_WS}/devel/setup.bash; fi" >> "${HOME}/.bashrc"

ENTRYPOINT ["/ros_entrypoint.sh"]
EXPOSE 8765
CMD ["bash"]

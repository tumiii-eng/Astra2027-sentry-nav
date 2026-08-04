FROM ghcr.io/smbu-polarbear-robotics-team/rmu_gazebo_simulator:1.0.0

ARG USERNAME=developer
ARG USER_UID=1000
ARG USER_GID=1000

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

RUN curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros2-latest-archive-keyring.gpg

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    ca-certificates \
    cmake \
    curl \
    git \
    gnupg \
    htop \
    libeigen3-dev \
    libomp-dev \
    lsb-release \
    python3-pip \
    python3-vcstool \
    sudo \
    unzip \
    vim \
    wget \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir gdown xmacro

RUN git clone --depth 1 https://github.com/koide3/small_gicp.git /tmp/small_gicp \
    && cmake -S /tmp/small_gicp -B /tmp/small_gicp/build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build /tmp/small_gicp/build --parallel "$(nproc)" \
    && cmake --install /tmp/small_gicp/build \
    && rm -rf /tmp/small_gicp

# Resolve all ROS dependencies from the host checkout without copying source
# into the image. BuildKit exposes src read-only for this single layer.
RUN --mount=type=bind,source=src,target=/tmp/ros_src,readonly \
    apt-get update \
    && rosdep install -r --from-paths /tmp/ros_src --ignore-src \
      --rosdistro humble -y \
    && rm -rf /var/lib/apt/lists/*

RUN groupadd --gid "${USER_GID}" "${USERNAME}" \
    && useradd --uid "${USER_UID}" --gid "${USER_GID}" -m -s /bin/bash "${USERNAME}" \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > "/etc/sudoers.d/${USERNAME}" \
    && chmod 0440 "/etc/sudoers.d/${USERNAME}" \
    && printf '%s\n' \
      'source /opt/ros/${ROS_DISTRO:-humble}/setup.bash' \
      '[[ -f /workspace/install/setup.bash ]] && source /workspace/install/setup.bash' \
      >> "/home/${USERNAME}/.bashrc" \
    && chown "${USER_UID}:${USER_GID}" "/home/${USERNAME}/.bashrc"

COPY docker/entrypoint.sh /usr/local/bin/pb2025-entrypoint
RUN chmod +x /usr/local/bin/pb2025-entrypoint

ENV DEBIAN_FRONTEND=
ENV USER=${USERNAME}
ENV HOME=/home/${USERNAME}
WORKDIR /workspace
USER ${USERNAME}
ENTRYPOINT ["/usr/local/bin/pb2025-entrypoint"]
CMD ["sleep", "infinity"]

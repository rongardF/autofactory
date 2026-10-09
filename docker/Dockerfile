FROM ros:jazzy-ros-base

ARG DEBIAN_FRONTEND=noninteractive

# ── System dependencies ────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-venv \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-jazzy-nav2-lifecycle-manager \
    sudo \
    # Mesa OpenGL/DRI libraries required by RViz2 and Qt-based GUI tools.
    # libgl1-mesa-dri provides both the hardware DRI drivers and the llvmpipe
    # software renderer (used when LIBGL_ALWAYS_SOFTWARE=1 is set).
    libgl1-mesa-dri \
    libglx-mesa0 \
    libegl-mesa0 \
    && rm -rf /var/lib/apt/lists/*

# ── Non-root user (used by production profile) ─────────────────────────────
ARG USERNAME=user
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid ${USER_GID} ${USERNAME} 2>/dev/null || true \
    && useradd --uid ${USER_UID} --gid ${USER_GID} --create-home ${USERNAME} 2>/dev/null || true \
    && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

# ── Workspace ──────────────────────────────────────────────────────────────
WORKDIR /workspaces/autofactory

# ── Python virtual environment ───────────────────────────────────────────
# --system-site-packages lets the venv see all ROS 2 apt-installed Python packages
# (rclpy, rclpy.lifecycle, etc.) without reinstalling them. Project packages
# install cleanly into /opt/venv, avoiding conflicts with Debian-managed copies.
RUN python3 -m venv /opt/venv --system-site-packages
ENV PATH="/opt/venv/bin:$PATH"

COPY requirements.txt requirements-dev.txt ./
RUN pip install --no-cache-dir -r requirements-dev.txt

# ── Shared Gazebo CMake helpers ─────────────────────────────────────────────
# gz_register_models() lets each ROS 2 package register its Gazebo model
# directory on GZ_SIM_RESOURCE_PATH (via an ament environment hook) without
# vendoring a copy of the helper. Packages locate it through GZ_CMAKE_MODULE_DIR:
#   list(APPEND CMAKE_MODULE_PATH "$ENV{GZ_CMAKE_MODULE_DIR}")
#   include(gz_register_models)
COPY docker/cmake/gz_register_models.cmake /usr/local/share/cmake/gz/
ENV GZ_CMAKE_MODULE_DIR=/usr/local/share/cmake/gz

# ── Entrypoint ─────────────────────────────────────────────────────────────
COPY docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

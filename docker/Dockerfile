FROM ros:jazzy-ros-base@sha256:066420e07f60aa18262f2479981def87ebcfcec42eefb0c0c57c4a46098348ca AS base

ARG DEBIAN_FRONTEND=noninteractive

# Preserve apt downloads in BuildKit's cache rather than in the final image.
RUN rm -f /etc/apt/apt.conf.d/docker-clean \
    && printf 'Binary::apt::APT::Keep-Downloaded-Packages "true";\n' > /etc/apt/apt.conf.d/keep-cache

# System and display tools
RUN --mount=type=cache,id=autofactory-apt,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,id=autofactory-apt-lists,target=/var/lib/apt/lists,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
    python3-pip=24.0+dfsg-1ubuntu1.3 \
    python3-venv=3.12.3-0ubuntu2.1 \
    python3-colcon-common-extensions=0.3.0-100 \
    python3-rosdep=0.27.0-1 \
    ros-jazzy-nav2-lifecycle-manager=1.3.13-1noble.20260921.025008 \
    sudo=1.9.15p5-3ubuntu5.24.04.4 \
    mesa-utils=9.0.0-2 \
    x11-utils=7.7+6build2 \
    xvfb=2:21.1.12-1ubuntu1.8 \
    x11vnc=0.9.16-10 \
    novnc=1:1.3.0-2 \
    websockify=0.10.0+dfsg1-5build2 \
    openbox=3.6.1-12build5 \
    libgl1-mesa-dri=25.2.8-0ubuntu0.24.04.4 \
    libglx-mesa0=25.2.8-0ubuntu0.24.04.4 \
    libegl-mesa0=25.2.8-0ubuntu0.24.04.4 \
    libglvnd0=1.7.0-1build1 \
    libgl1=1.7.0-1build1 \
    libglx0=1.7.0-1build1 \
    libegl1=1.7.0-1build1 \
    libgles2=1.7.0-1build1 \
    libxcb-cursor0=0.1.4-1build1 \
    libxcb-xinerama0=1.15-1ubuntu2 \
    libxcb-xinput0=1.15-1ubuntu2 \
    wget=1.21.4-1ubuntu4.5

FROM base AS ros-dependencies

# ROS workspace dependencies (update alongside package.xml)
RUN --mount=type=cache,id=autofactory-apt,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,id=autofactory-apt-lists,target=/var/lib/apt/lists,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
    python3-numpy=1:1.26.4+ds-6ubuntu1 \
    python3-opencv=4.6.0+dfsg-13.1ubuntu1 \
    python3-pydantic=1.10.14-1 \
    python3-pytest=7.4.4-1 \
    ros-jazzy-action-msgs=2.0.4-1noble.20260911.052334 \
    ros-jazzy-ament-cmake=2.5.6-2noble.20260225.222913 \
    ros-jazzy-ament-cmake-pytest=2.5.6-2noble.20260225.145739 \
    ros-jazzy-ament-cmake-python=2.5.6-2noble.20260225.042312 \
    ros-jazzy-ament-index-python=1.8.4-1noble.20260519.010916 \
    ros-jazzy-ament-lint-auto=0.17.5-1noble.20260410.080333 \
    ros-jazzy-ament-lint-common=0.17.5-1noble.20260410.114154 \
    ros-jazzy-builtin-interfaces=2.0.4-1noble.20260911.051559 \
    ros-jazzy-camera-info-manager=5.1.8-1noble.20260912.163210 \
    ros-jazzy-cv-bridge=4.1.0-1noble.20260911.143610 \
    ros-jazzy-diagnostic-updater=4.2.7-1noble.20260912.163704 \
    ros-jazzy-geometry-msgs=5.3.8-1noble.20260911.102920 \
    ros-jazzy-gz-common-vendor=0.0.10-1noble.20260905.033536 \
    ros-jazzy-gz-math-vendor=0.0.11-1noble.20260905.031649 \
    ros-jazzy-gz-msgs-vendor=0.0.8-1noble.20260905.033532 \
    ros-jazzy-gz-plugin-vendor=0.0.5-1noble.20260820.145930 \
    ros-jazzy-gz-sim-vendor=0.0.13-1noble.20260905.073557 \
    ros-jazzy-gz-transport-vendor=0.0.9-1noble.20260905.065154 \
    ros-jazzy-ros-gz-interfaces=1.0.24-1noble.20260911.110630 \
    ros-jazzy-gz-ros2-control=1.2.20-1noble.20261006.040504 \
    ros-jazzy-image-geometry=4.1.0-1noble.20260911.143731 \
    ros-jazzy-image-transport=5.1.8-1noble.20260912.165731 \
    ros-jazzy-launch=3.4.12-1noble.20260920.232232 \
    ros-jazzy-launch-ros=0.26.13-1noble.20260920.233704 \
    ros-jazzy-lifecycle-msgs=2.0.4-1noble.20260911.054418 \
    ros-jazzy-moveit-configs-utils=2.12.4-1noble.20260920.234447 \
    ros-jazzy-moveit-kinematics=2.12.4-1noble.20260915.201542 \
    ros-jazzy-moveit-msgs=2.6.0-1noble.20260911.142759 \
    ros-jazzy-moveit-planners=2.12.4-1noble.20260915.204614 \
    ros-jazzy-moveit-planners-chomp=2.12.4-1noble.20260915.194935 \
    ros-jazzy-moveit-ros-move-group=2.12.4-1noble.20260915.202214 \
    ros-jazzy-moveit-ros-planning-interface=2.12.4-1noble.20260915.203504 \
    ros-jazzy-moveit-ros-visualization=2.12.4-1noble.20260915.204804 \
    ros-jazzy-moveit-servo=2.12.4-1noble.20261006.040508 \
    ros-jazzy-moveit-simple-controller-manager=2.12.4-1noble.20260915.194321 \
    ros-jazzy-pcl-ros=2.6.5-1noble.20260916.065720 \
    ros-jazzy-rcl-interfaces=2.0.4-1noble.20260911.093433 \
    ros-jazzy-rclcpp=28.1.22-1noble.20260911.105434 \
    ros-jazzy-rclcpp-action=28.1.22-1noble.20260912.162328 \
    ros-jazzy-rclcpp-components=28.1.22-1noble.20260911.115213 \
    ros-jazzy-rclpy=7.1.12-1noble.20260912.162354 \
    ros-jazzy-rcutils=6.7.6-1noble.20260911.045439 \
    ros-jazzy-ros-gz-bridge=1.0.24-1noble.20260920.234644 \
    ros-jazzy-ros-gz-sim=1.0.24-1noble.20261006.035056 \
    ros-jazzy-rosidl-default-generators=1.6.1-1noble.20260911.052510 \
    ros-jazzy-rosidl-default-runtime=1.6.1-1noble.20260911.052508 \
    ros-jazzy-rqt-graph=1.5.6-1noble.20260912.181613 \
    ros-jazzy-rqt-image-view=1.3.0-2noble.20260912.180617 \
    ros-jazzy-sensor-msgs=5.3.8-1noble.20260911.140426 \
    ros-jazzy-shape-msgs=5.3.8-1noble.20260911.113155 \
    ros-jazzy-std-msgs=5.3.8-1noble.20260911.094424 \
    ros-jazzy-std-srvs=5.3.8-1noble.20260911.100759 \
    ros-jazzy-tf2-ros=0.36.23-1noble.20260915.173839 \
    ros-jazzy-ur-msgs=2.6.0-1noble.20260911.142433 \
    ros-jazzy-ur-robot-driver=3.9.0-1noble.20261006.040539 \
    ros-jazzy-xacro=2.1.1-1noble.20260519.011123

# Pin the RViz executable as well as the libraries used by MoveIt.
RUN --mount=type=cache,id=autofactory-apt,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,id=autofactory-apt-lists,target=/var/lib/apt/lists,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-rviz2=14.1.24-1noble.20260915.204213

FROM ros-dependencies AS camera-sdk

# Pylon 25.10.2 and Blaze 1.7.3. Versioned archives with verified checksums.
RUN --mount=type=cache,id=autofactory-sdk,target=/var/cache/autofactory,sharing=locked \
    --mount=type=cache,id=autofactory-apt,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,id=autofactory-apt-lists,target=/var/lib/apt/lists,sharing=locked \
    set -eu; \
    cd /var/cache/autofactory; \
    pylon=43f19f2bcc2ad9438740b99bbd3063fba3a8ab1f87c4361b183e6feaca310f65-pylon.tar.gz; \
    blaze=19e1c8e22b0eaff11dbdaa40065a1a8f68af1576f5173ef259a4d8de59dced35-blaze.tar.gz; \
    if [ ! -f "$pylon" ]; then wget -O "$pylon.partial" 'https://downloadbsl.blob.core.windows.net/software/pylon%2025.10.2/pylon-25.10.2_linux-x86_64_debs.tar.gz'; mv "$pylon.partial" "$pylon"; fi; \
    if [ ! -f "$blaze" ]; then wget -O "$blaze.partial" 'https://downloads-ctf.baslerweb.com/dg51pdwahxgw/cmMx43uVISAcg4t7IqrrF/83487e4dcccac2c0644ee5fadc4c323c/pylon-supplementary-package-for-blaze-1.7.3.73dbe706a_x86_64_setup.tar.gz'; mv "$blaze.partial" "$blaze"; fi; \
    echo "43f19f2bcc2ad9438740b99bbd3063fba3a8ab1f87c4361b183e6feaca310f65  $pylon" | sha256sum -c -; \
    echo "19e1c8e22b0eaff11dbdaa40065a1a8f68af1576f5173ef259a4d8de59dced35  $blaze" | sha256sum -c -; \
    mkdir -p /tmp/pylon-setup; \
    tar -xf "$pylon" -C /tmp/pylon-setup; \
    tar -xf "$blaze" -C /tmp/pylon-setup; \
    apt-get update; \
    apt-get install -y --no-install-recommends /tmp/pylon-setup/pylon_25.10.2-deb0_amd64.deb; \
    tar -xf /tmp/pylon-setup/pylon-supplementary-package-for-blaze-1.7.3.73dbe706a_x86_64_setup/pylon-supplementary-package-for-blaze-1.7.3.73dbe706a_x86_64.tar.gz -C /opt/pylon; \
    rm -rf /tmp/pylon-setup

FROM camera-sdk AS python-dependencies

# ── Workspace ──────────────────────────────────────────────────────────────
WORKDIR /workspaces/autofactory

# ── Python virtual environment ───────────────────────────────────────────
# --system-site-packages lets the venv see all ROS 2 apt-installed Python packages
# (rclpy, rclpy.lifecycle, etc.) without reinstalling them. Project packages
# install cleanly into /opt/venv, avoiding conflicts with Debian-managed copies.
RUN python3 -m venv /opt/venv --system-site-packages
ENV PATH="/opt/venv/bin:$PATH"

COPY requirements.txt requirements-dev.txt ./
RUN --mount=type=cache,id=autofactory-pip,target=/root/.cache/pip \
    pip install -r requirements-dev.txt

FROM python-dependencies AS development

# Advertise GPU/graphics capabilities to the NVIDIA Container Toolkit so it
# mounts the driver libraries and /dev/nvidia* devices at container start.
ENV NVIDIA_VISIBLE_DEVICES=void
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute

# The toolkit only mounts the driver's versioned .so libraries at runtime; the
# glvnd ICD json (version-independent) is not injected and must ship in the
# image itself, or glvnd silently falls back to the Mesa ICD (software render).
RUN mkdir -p /usr/share/glvnd/egl_vendor.d /usr/share/glvnd/glx_vendor.d \
    && echo '{"file_format_version":"1.0.0","ICD":{"library_path":"libEGL_nvidia.so.0"}}' \
    > /usr/share/glvnd/egl_vendor.d/10_nvidia.json \
    && echo '{"file_format_version":"1.0.0","library_path":"libGLX_nvidia.so.0"}' \
    > /usr/share/glvnd/glx_vendor.d/10_nvidia.json

# ── Development account (also UID 1000 for the production profile) ─────────
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000
COPY docker/create-dev-user.sh /usr/local/bin/create-dev-user
RUN bash /usr/local/bin/create-dev-user "${USERNAME}" "${USER_UID}" "${USER_GID}"

# ── Shared Gazebo CMake helpers ─────────────────────────────────────────────
# gz_register_models() lets each ROS 2 package register its Gazebo model
# directory on GZ_SIM_RESOURCE_PATH (via an ament environment hook) without
# vendoring a copy of the helper. Packages locate it through GZ_CMAKE_MODULE_DIR:
#   list(APPEND CMAKE_MODULE_PATH "$ENV{GZ_CMAKE_MODULE_DIR}")
#   include(gz_register_models)
COPY docker/cmake/gz_register_models.cmake /usr/local/share/cmake/gz/
ENV GZ_CMAKE_MODULE_DIR=/usr/local/share/cmake/gz

# ── Entrypoint ─────────────────────────────────────────────────────────────
COPY docker/autofactory-desktop.sh /usr/local/bin/autofactory-desktop
COPY docker/setup.bash /usr/local/share/autofactory/setup.bash
COPY docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /usr/local/bin/autofactory-desktop /ros_entrypoint.sh \
    && echo '. /usr/local/share/autofactory/setup.bash' >> /etc/bash.bashrc \
    && ln -s /usr/local/share/autofactory/setup.bash /etc/profile.d/autofactory.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

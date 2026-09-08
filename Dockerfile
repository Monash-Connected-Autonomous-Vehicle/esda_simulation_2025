FROM osrf/ros:humble-desktop-full

ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV ROS_DISTRO=humble

# Optional heavy ML extras for the alternative lane detectors.
#   INSTALL_TORCH=true       -> lane_detection_twinlite.py (TwinLiteNet+, CPU PyTorch)
#   INSTALL_TENSORFLOW=true  -> lane_detection_FCN.py (Keras .h5 model)
# The classic CV detector (lane_detection.py) needs neither.
ARG INSTALL_TORCH=false
ARG INSTALL_TENSORFLOW=false

USER root

# ---------------------------------------------------------------------------
# Dev tools, locale, and the shell utilities ui_launch.py shells out to
# (xterm windows per module, killall/pkill in "Kill All Processes")
# ---------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        curl \
        gnupg \
        lsb-release \
        software-properties-common \
        locales \
        sudo \
        wget \
        git \
        nano \
        vim \
        gedit \
        xterm \
        terminator \
        psmisc \
        procps \
        iproute2 \
        net-tools \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && add-apt-repository universe -y \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# OSRF Gazebo repository
# ---------------------------------------------------------------------------
RUN curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
        -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
       http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
       > /etc/apt/sources.list.d/gazebo-stable.list

# ---------------------------------------------------------------------------
# Simulation + navigation stack
#
# Gazebo Fortress (Ignition), not Garden/Harmonic: worlds/*.sdf load
# libignition-gazebo-*-system.so plugins and the robot xacro uses the
# gz_ros2_control-system plugin, both of which are the Fortress ABI.
# ---------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        ignition-fortress \
        ros-humble-ros-gz \
        ros-humble-ros-gz-sim \
        ros-humble-ros-gz-bridge \
        ros-humble-ros-gz-image \
        ros-humble-ros2-control \
        ros-humble-ros2-controllers \
        ros-humble-gz-ros2-control \
        ros-humble-controller-manager \
        ros-humble-diff-drive-controller \
        ros-humble-joint-state-broadcaster \
        ros-humble-navigation2 \
        ros-humble-nav2-bringup \
        ros-humble-nav2-simple-commander \
        ros-humble-nav2-msgs \
        ros-humble-slam-toolbox \
        ros-humble-robot-localization \
        ros-humble-robot-state-publisher \
        ros-humble-joint-state-publisher \
        ros-humble-joint-state-publisher-gui \
        ros-humble-xacro \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# Perception, TF, teleop, and the LiDAR driver used on the real robot
# ---------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-cv-bridge \
        ros-humble-image-transport \
        ros-humble-vision-opencv \
        ros-humble-pcl-conversions \
        ros-humble-tf2 \
        ros-humble-tf2-ros \
        ros-humble-tf2-geometry-msgs \
        ros-humble-tf2-tools \
        ros-humble-tf-transformations \
        ros-humble-sensor-msgs-py \
        ros-humble-velodyne \
        ros-humble-velodyne-driver \
        ros-humble-velodyne-pointcloud \
        ros-humble-velodyne-laserscan \
        ros-humble-joy \
        ros-humble-teleop-twist-joy \
        ros-humble-teleop-twist-keyboard \
        ros-humble-rmw-fastrtps-cpp \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# Build tooling, Python deps, software GL, and libserial for the
# esda_hardware_2025 ros2_control plugin (links against -lserial)
# ---------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-pip \
        python3-tk \
        python3-opencv \
        python3-numpy \
        python3-yaml \
        python3-transforms3d \
        libserial-dev \
        libgl1-mesa-dri \
        libgl1-mesa-glx \
        libglu1-mesa \
        mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# ui_launch.py / ui_launch_real_robot.py GUI toolkit
RUN pip3 install --no-cache-dir customtkinter

# Optional: TwinLiteNet+ detector (CPU-only PyTorch, ~1 GB)
RUN if [ "${INSTALL_TORCH}" = "true" ]; then \
        apt-get update \
        && apt-get install -y --no-install-recommends python3-torch python3-torchvision \
        && rm -rf /var/lib/apt/lists/*; \
    fi

# Optional: FCN detector (Keras/TensorFlow, ~600 MB)
RUN if [ "${INSTALL_TENSORFLOW}" = "true" ]; then \
        pip3 install --no-cache-dir tensorflow-cpu; \
    fi

RUN rosdep init 2>/dev/null || true \
    && rosdep update --rosdistro ${ROS_DISTRO}

# ---------------------------------------------------------------------------
# Non-root user with sudo
# ---------------------------------------------------------------------------
RUN adduser --disabled-password --gecos '' user \
    && adduser user sudo \
    && passwd -d user \
    && echo "user ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/user

# Shared memory is unreliable across the Docker/WSL boundary; force the
# UDP-only Fast DDS profile the repo already ships.
RUN { \
      echo "source /opt/ros/${ROS_DISTRO}/setup.bash"; \
      echo "if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash; fi"; \
      echo "export FASTRTPS_DEFAULT_PROFILES_FILE=/ros2_ws/src/esda_simulation_2025/config/fastdds_noshm.xml"; \
      echo "export GZ_SIM_RESOURCE_PATH=/ros2_ws/install/esda_simulation_2025/share:/ros2_ws/src/esda_simulation_2025/worlds:/ros2_ws/src/esda_simulation_2025/worlds/models"; \
      echo "cd /ros2_ws"; \
    } >> /home/user/.bashrc \
    && chown user:user /home/user/.bashrc

ENV FASTRTPS_DEFAULT_PROFILES_FILE=/ros2_ws/src/esda_simulation_2025/config/fastdds_noshm.xml
ENV QT_X11_NO_MITSHM=1

USER user
WORKDIR /ros2_ws
COPY --chown=user:user src/ src/

# Prebuild so a fresh container is immediately runnable. When the repo is
# bind-mounted over /ros2_ws (see docker-compose.yml) this layer is shadowed
# and you rebuild inside the container instead.
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build --symlink-install

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

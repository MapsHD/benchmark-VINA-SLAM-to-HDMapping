FROM ubuntu:22.04

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

# ── Base tools ────────────────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    build-essential \
    cmake \
    git \
    ca-certificates \
    wget \
    pkg-config \
    libeigen3-dev \
    libpcl-dev \
    libapr1-dev \
    nlohmann-json3-dev \
    tmux \
    python3-pip \
    python3-yaml \
    && rm -rf /var/lib/apt/lists/*

# ── ROS 2 Humble ─────────────────────────────────────────────────────────────
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-desktop \
    ros-humble-pcl-ros \
    ros-humble-pcl-conversions \
    ros-humble-tf2-eigen \
    ros-humble-rosbag2-cpp \
    ros-humble-rosbag2-storage-default-plugins \
    ros-humble-ament-lint-auto \
    ros-humble-ament-lint-common \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# ── rosbags (used to convert ROS 1 bags to ROS 2, if needed) ─────────────────
RUN pip3 install --no-cache-dir "rosbags==0.9.22"

# ── Livox-SDK2 ───────────────────────────────────────────────────────────────
# livox_ros_driver2 links its shared library from /usr/local/lib. VINA-SLAM
# only needs the driver's CustomMsg, but the driver package builds as a whole.
COPY ./src/Livox-SDK2 /tmp/Livox-SDK2
RUN cmake -S /tmp/Livox-SDK2 -B /tmp/Livox-SDK2/build -DCMAKE_BUILD_TYPE=Release && \
    cmake --build /tmp/Livox-SDK2/build -j4 && \
    cmake --install /tmp/Livox-SDK2/build && \
    ldconfig && \
    test -f /usr/local/lib/liblivox_lidar_sdk_shared.so && \
    rm -rf /tmp/Livox-SDK2

# ── Build colcon workspace (livox_ros_driver2, VINA-SLAM, converter) ─────────
WORKDIR /ros2_ws

COPY ./src/livox_ros_driver2       ./src/livox_ros_driver2
COPY ./src/VINA-SLAM               ./src/VINA-SLAM
COPY ./src/vina-slam-to-hdmapping  ./src/vina-slam-to-hdmapping

# livox_ros_driver2's build.sh for ROS 2: the ROS 2 package manifest replaces
# package.xml, and colcon gets ROS_EDITION and DISTRO_ROS (below).
RUN cp -f src/livox_ros_driver2/package_ROS2.xml src/livox_ros_driver2/package.xml

# Benchmark parameter file: VINA-SLAM's mid360.yaml with the local BA on
# (if_BA: 1). The author's file has if_BA: 0, which runs the EKF only, without
# the sliding-window BA and its normal-consistency factor, VINA-SLAM's own
# contribution. The author's other configs (HILTI, velodyne, robosense,
# outdoor_fly, compus_elevator) all use if_BA: 1. Nothing else changes.
RUN sed -E 's/^(\s*if_BA:\s*)0\b/\11/' src/VINA-SLAM/config/mid360.yaml > src/VINA-SLAM/config/mid360_bench.yaml && \
    grep -qE '^\s*if_BA:\s*1\b' src/VINA-SLAM/config/mid360_bench.yaml && \
    test "$(diff src/VINA-SLAM/config/mid360.yaml src/VINA-SLAM/config/mid360_bench.yaml | grep -c '^[<>]')" = 2

# Benchmark launch file and RViz view added into the vina_slam package;
# preflight check used by the run script.
COPY ./overlay/launch/  ./src/VINA-SLAM/launch/
COPY ./overlay/rviz/    ./src/VINA-SLAM/rviz_cfg/
COPY ./overlay/scripts/ /ros2_ws/scripts/

# The final guard fails the image build loudly if an executable is missing.
RUN source /opt/ros/humble/setup.bash && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
      -DROS_EDITION=ROS2 -DDISTRO_ROS=humble && \
    test -f /ros2_ws/install/vina_slam/lib/vina_slam/vina_slam && \
    test -f /ros2_ws/install/vina_slam/share/vina_slam/launch/vina_slam_bench.launch.py && \
    test -f /ros2_ws/install/vina_slam/share/vina_slam/rviz_cfg/vina_slam_bench.rviz && \
    test -f /ros2_ws/install/vina_slam/share/vina_slam/config/mid360_bench.yaml && \
    test -f /ros2_ws/install/vina_slam_to_hdmapping/lib/vina_slam_to_hdmapping/listener && \
    echo "[build] vina_slam, bench launch, RViz view, bench config and converter present"

# ── Non-root user ─────────────────────────────────────────────────────────────
ARG UID=1000
ARG GID=1000
RUN groupadd -g $GID ros && \
    useradd -m -u $UID -g $GID -s /bin/bash ros && \
    chown -R $UID:$GID /ros2_ws

RUN echo "source /opt/ros/humble/setup.bash"   >> /root/.bashrc && \
    echo "source /ros2_ws/install/setup.bash"  >> /root/.bashrc && \
    echo "source /opt/ros/humble/setup.bash"   >> /home/ros/.bashrc && \
    echo "source /ros2_ws/install/setup.bash"  >> /home/ros/.bashrc

CMD ["bash"]

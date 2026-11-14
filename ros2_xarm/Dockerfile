FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# 1. Base tools, Gazebo classic, ros2_control, controllers, and MoveIt 2
RUN apt-get update && apt-get install -y \
    git \
    wget \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-joint-state-broadcaster \
    ros-humble-joint-trajectory-controller \
    ros-humble-moveit \
    ros-humble-xacro \
    mesa-utils \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*

# 2. Setup workspace and clone official xarm_ros2 recursively (with C++ SDK)
WORKDIR /ros2_ws/src
RUN git clone --recursive -b humble --depth 1 https://github.com/xArm-Developer/xarm_ros2.git

# 3. Resolve rosdep dependencies
WORKDIR /ros2_ws
RUN apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src --rosdistro humble -y && \
    rm -rf /var/lib/apt/lists/*

# 4. Build workspace with limited workers to avoid OOM
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    MAKEFLAGS='-j2' colcon build --symlink-install --parallel-workers 2"

# 5. Automatically source underlay and overlay in bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

WORKDIR /ros2_ws
CMD ["/bin/bash"]

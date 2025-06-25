ARG BASE_IMAGE=ghcr.io/watonomous/robot_base/base:humble-ubuntu22.04

################################ Source ################################
FROM ${BASE_IMAGE} AS source

RUN mkdir -p /usr/share/keyrings && \
    rm -f /etc/apt/sources.list.d/ros2-latest.list && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | \
    gpg --batch --yes --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" \
    > /etc/apt/sources.list.d/ros2.list

WORKDIR ${AMENT_WS}/src

# Clean up and update apt-get, then update rosdep
RUN sudo apt-get clean && \
    sudo apt-get update && \
    sudo rosdep update

COPY src/robot .

# Scan for rosdeps
RUN apt-get -qq update && rosdep update && \
    rosdep install --from-paths . --ignore-src -r -s \
    | grep 'apt-get install' \
    | awk '{print $3}' \
    | sort  > /tmp/colcon_install_list

################################# Dependencies ################################
FROM ${BASE_IMAGE} AS dependencies

# Remove conflicting ROS 2 APT source and update GPG key
RUN rm -f /etc/apt/sources.list.d/ros2-latest.list && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | \
    gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" \
    > /etc/apt/sources.list.d/ros2.list

# Clean up and update apt-get, then update rosdep
RUN sudo apt-get clean && \
    sudo apt-get update && \
    sudo rosdep update

# ADD MORE DEPENDENCIES HERE
RUN rm -rf /var/lib/apt/lists/* && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
    curl \
    ros-humble-ros2bag \
    ros-humble-rosbag2 \
    ros-humble-rosbag2-storage \
    ros-humble-rosbag2-transport \
    ros-humble-rosbag2-compression \
    ros-humble-rosbag2-interfaces \
    ros-humble-rosbag2-storage-default-plugins \
    ros-humble-foxglove-msgs \
    && rm -rf /var/lib/apt/lists/*

RUN sudo apt-get install libeigen3-dev

# Install Rosdep requirements
COPY --from=source /tmp/colcon_install_list /tmp/colcon_install_list
RUN apt-get update && \
    apt-get install -y --no-install-recommends $(cat /tmp/colcon_install_list) || \
    apt-get install -y --no-install-recommends --fix-missing

# Copy in source code from source stage
WORKDIR ${AMENT_WS}
COPY --from=source ${AMENT_WS}/src src

# Dependency Cleanup
WORKDIR /
RUN apt-get -qq autoremove -y && apt-get -qq autoclean && apt-get -qq clean && \
    rm -rf /root/* /root/.ros /tmp/* /var/lib/apt/lists/* /usr/share/doc/*

################################ Build ################################
FROM dependencies AS build

RUN mkdir -p /usr/share/keyrings && \
    rm -f /etc/apt/sources.list.d/ros2-latest.list && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | \
    gpg --batch --yes --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" \
    > /etc/apt/sources.list.d/ros2.list

# Clean up and update apt-get, then update rosdep
RUN sudo apt-get clean && \
    sudo apt-get update && \
    sudo rosdep update

# Build ROS2 packages
WORKDIR ${AMENT_WS}
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build \
    --cmake-args -DCMAKE_BUILD_TYPE=Release --install-base ${WATONOMOUS_INSTALL}

# Source and Build Artifact Cleanup
RUN rm -rf src/* build/* devel/* install/* log/*

COPY docker/wato_ros_entrypoint.sh ${AMENT_WS}/wato_ros_entrypoint.sh
ENTRYPOINT ["./wato_ros_entrypoint.sh"]

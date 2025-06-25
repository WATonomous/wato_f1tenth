ARG BASE_IMAGE=ghcr.io/watonomous/robot_base/base:humble-ubuntu22.04

################################ Source ################################
FROM ${BASE_IMAGE} AS source

WORKDIR ${AMENT_WS}/src

# Copy in source code 
COPY src/wato_msgs wato_msgs

# Scan for rosdeps
RUN apt-get -qq update && rosdep update && \
    rosdep install --from-paths . --ignore-src -r -s \
    | grep 'apt-get install' \
    | awk '{print $3}' \
    | sort  > /tmp/colcon_install_list

################################# Dependencies ################################
# ...existing code...

FROM ${BASE_IMAGE} AS dependencies

# Remove conflicting ROS 2 APT source and update GPG key
RUN rm -f /etc/apt/sources.list.d/ros2*.list && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | \
    gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros2.list

# Clean up and update apt-get, then update rosdep
RUN apt-get clean && \
    apt-get update && \
    rosdep update

# Install Foxglove Deps
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    ros-humble-ros2bag || true && \
    apt-get install -y --no-install-recommends \
    ros-humble-rosbag2-storage-default-plugins || true && \
    apt-get install -y --no-install-recommends \
    ros-humble-rosbag2-compression || true && \
    apt-get install -y --no-install-recommends \
    ros-humble-rosbag2-transport || true && \
    apt-get install -y --no-install-recommends \
    ros-humble-foxglove-msgs || true && \
    rm -rf /var/lib/apt/lists/*

COPY --from=source /tmp/colcon_install_list /tmp/colcon_install_list
RUN apt-fast install -qq -y --no-install-recommends $(cat /tmp/colcon_install_list)

WORKDIR ${AMENT_WS}
COPY --from=source ${AMENT_WS}/src src

WORKDIR /
RUN apt-get -qq autoremove -y && apt-get -qq autoclean && apt-get -qq clean && \
    rm -rf /root/* /root/.ros /tmp/* /var/lib/apt/lists/* /usr/share/doc/*

FROM dependencies AS build

# Build ROS2 packages
WORKDIR ${AMENT_WS}
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build \
    --cmake-args -DCMAKE_BUILD_TYPE=Release --install-base ${WATONOMOUS_INSTALL}

# Source and Build Artifact Cleanup 
RUN rm -rf src/* build/* devel/* install/* log/*

# Entrypoint will run before any CMD on launch. Sources ~/opt/<ROS_DISTRO>/setup.bash and ~/ament_ws/install/setup.bash
COPY docker/wato_ros_entrypoint.sh ${AMENT_WS}/wato_ros_entrypoint.sh
ENTRYPOINT ["./wato_ros_entrypoint.sh"]

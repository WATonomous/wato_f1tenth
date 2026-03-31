FROM docker.io/arm64v8/ubuntu:22.04

ARG DEBIAN_FRONTEND=noninteractive

# Basic dependencies
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        build-essential \
        ca-certificates \
        curl \
        gnupg2 \
        lsb-release \
        software-properties-common \
        apt-transport-https \
        wget \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2 Humble
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
        http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
        | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-ros-base \
        ros-dev-tools \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Install colcon and rosdep
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions \
        python3-rosdep \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && rosdep update

# Install Foxglove deps
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-ros2bag \
        ros-humble-rosbag2-cpp \
        ros-humble-rosbag2-storage \
        ros-humble-rosbag2-transport \
        ros-humble-rosbag2-compression \
        ros-humble-rosbag2-storage-default-plugins \
        ros-humble-rosbag2-storage-mcap \
        ros-humble-foxglove-msgs \
        ros-humble-cv-bridge \
        ros-humble-rosbridge-server \
        ros-humble-topic-tools \
        ros-humble-vision-msgs \
        ros-humble-foxglove-bridge \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Add universe repo for apt-fast
RUN apt-add-repository universe

# Copy in source and build
RUN mkdir -p /home/ros_user/ros_ws
WORKDIR /home/ros_user/ros_ws
COPY src/wato_msgs src

# Install rosdep requirements
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    rosdep install -r --from-paths src --ignore-src --rosdistro humble -y"

# Build
RUN /bin/bash -c "source /opt/ros/humble/setup.sh && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release"

# Cleanup
RUN rm -rf src/* build/* devel/* install/* log/*

# Bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc

COPY docker/wato_ros_entrypoint.sh /home/ros_user/ros_ws/wato_ros_entrypoint.sh
ENTRYPOINT ["./wato_ros_entrypoint.sh"]

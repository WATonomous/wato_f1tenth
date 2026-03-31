FROM docker.io/arm64v8/ubuntu:22.04

ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all

#bare minimu nvidia  
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && \
    apt-get install -qq -y --no-install-recommends \
        build-essential \
        ca-certificates \
        cmake \
        curl \
        gnupg2 \
        libglu1-mesa-dev \
        libglvnd-dev \
        libgtk-3-0 \
        libudev1 \
        libvulkan1 \
        python3 \
        python3-distutils \
        python3-numpy \
        python3-pexpect \
        python3-pip \
        sudo \
        wget \
        && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*


# EGL
RUN echo "/usr/lib/aarch64-linux-gnu/tegra" >> /etc/ld.so.conf.d/nvidia-tegra.conf && \
    echo "/usr/lib/aarch64-linux-gnu/tegra-egl" >> /etc/ld.so.conf.d/nvidia-tegra.conf
RUN rm -rf /usr/share/glvnd/egl_vendor.d && \
    mkdir -p /usr/share/glvnd/egl_vendor.d/ && echo '\
{\
    "file_format_version" : "1.0.0",\
    "ICD" : {\
        "library_path" : "libEGL_nvidia.so.0"\
    }\
}' > /usr/share/glvnd/egl_vendor.d/10_nvidia.json
RUN mkdir -p /usr/share/egl/egl_external_platform.d/ && echo '\
{\
    "file_format_version" : "1.0.0",\
    "ICD" : {\
        "library_path" : "libnvidia-egl-wayland.so.1"\
    }\
}' > /usr/share/egl/egl_external_platform.d/nvidia_wayland.json

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/jetson-ota-public.gpg] https://repo.download.nvidia.com/jetson/common r36.4 main" >> /etc/apt/sources.list.d/nvidia-l4t-apt-source.list && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/jetson-ota-public.gpg] https://repo.download.nvidia.com/jetson/t234 r36.4 main" >> /etc/apt/sources.list.d/nvidia-l4t-apt-source.list && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/jetson-ota-public.gpg] https://repo.download.nvidia.com/jetson/ffmpeg r36.4 main" >> /etc/apt/sources.list.d/nvidia-l4t-apt-source.list
RUN wget -O /etc/jetson-ota-public.key https://gitlab.com/nvidia/container-images/l4t-base/-/raw/master/jetson-ota-public.key && \
    cat /etc/jetson-ota-public.key | gpg --dearmor -o /usr/share/keyrings/jetson-ota-public.gpg


# install ROS2 Humble
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-desktop \
        ros-humble-rmw-cyclonedds-cpp \
        ros-humble-rosbag2-storage-mcap \
        ros-dev-tools \
        && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# install colcon and rosdep
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions \
        python3-rosdep \
        && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# initialize rosdep
RUN sudo rosdep init && \
    rosdep update

#add the slam toolbox, localizaiton and rviz2
RUN sudo apt-get update
#RUN sudo apt-get install -y ros-humble-rviz2
RUN sudo apt-get install -y ros-humble-navigation2
RUN sudo apt-get install -y ros-humble-slam-toolbox

#add controler support to the container
RUN sudo apt-get install -y ros-humble-joy 
RUN sudo apt-get install -y jstest-gtk
RUN mkdir -p /root/.config/jstest-gtk

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc

#copy in the code 
RUN mkdir -p /home/ros_user/ros_ws
WORKDIR /home/ros_user/ros_ws
COPY src/robot src

RUN sudo apt-get install -y pip
RUN sudo pip3 install cython

RUN mkdir -p /tmp/build && \
    cd /tmp/build && \
    git clone https://github.com/f1tenth/range_libc.git && \
    cd range_libc/pywrapper && \
    python3 setup.py install && \
    cd / && rm -rf /tmp/build

#use ros dep to solve the remaning dependencies 
RUN cd /home/ros_user/ros_ws
#RUN rosdep install -r --from-paths src --ignore-src --rosdistro humble -y
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && apt-get update && \
    rosdep install -r --from-paths src --ignore-src --rosdistro humble -y"

#build the ros 2 project 
RUN . /opt/ros/humble/setup.sh && \
    colcon build

#clean up the directory
RUN rm -rf src/* build/* devel/* install/* log/*

#entry point
COPY docker/wato_ros_entrypoint.sh /home/ros_user/ros_ws/wato_ros_entrypoint.sh
RUN chmod +x /home/ros_user/ros_ws/wato_ros_entrypoint.sh
ENTRYPOINT ["/home/ros_user/ros_ws/wato_ros_entrypoint.sh"]

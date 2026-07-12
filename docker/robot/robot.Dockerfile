# ============================================================================
# Base: NVIDIA JetPack container (L4T r36.4 / JetPack 6.x, Ubuntu 22.04 arm64)
# Ships with CUDA, cuDNN, TensorRT, VPI, the tegra libs, EGL/GLVND config,
# and the repo.download.nvidia.com/jetson apt sources already set up.
# -> replaces the old manual EGL vendor files + jetson OTA key/repo sections.
# Run with: docker run --runtime nvidia ...
# ============================================================================

# Source of the CUDA-enabled JAX build for the MPPI controller (mppi_example).
# Pulled in via COPY below rather than pip: the pip index this image's own
# config points at (jetson.webredirect.org -> pypi.jetson-ai-lab.dev) is dead,
# and the live pypi.jetson-ai-lab.io mirror only carries Google's newer generic
# aarch64+CUDA wheels (built for datacenter ARM, not verified against Orin's
# sm_87). This exact image was verified on this hardware: jax.devices() ->
# [CudaDevice(id=0)].
FROM dustynv/jax:r36.4.0 AS jax_src

FROM nvcr.io/nvidia/l4t-jetpack:r36.4.0

ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all

# GPU compute capability for range_libc:
#   Orin (t234, your target) = sm_87 | Xavier = sm_72 | old Nano/TX1 = sm_53
ARG CUDA_ARCH=sm_87

# Make the CUDA toolkit from the base image visible to builds (nvcc etc.)
ENV CUDAHOME=/usr/local/cuda
ENV CUDA_HOME=/usr/local/cuda
ENV PATH=/usr/local/cuda/bin:${PATH}
ENV LD_LIBRARY_PATH=/usr/local/cuda/lib64:${LD_LIBRARY_PATH}

# bare minimum tooling (all NVIDIA deps now come from the base image)
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && \
    apt-get install -qq -y --no-install-recommends \
        build-essential \
        ca-certificates \
        cmake \
        curl \
        git \
        gnupg2 \
        libgtk-3-0 \
        libudev1 \
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

# install ROS2 Humble (prebuilt debs -- jammy/arm64 is a Tier 1 platform)
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
RUN rosdep init && \
    rosdep update

# add the slam toolbox, localization and rviz2 (all prebuilt debs)
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ros-humble-navigation2 \
        ros-humble-slam-toolbox

# add controller support to the container
RUN apt-get install -y --no-install-recommends \
        ros-humble-joy \
        jstest-gtk
RUN mkdir -p /root/.config/jstest-gtk

# Realsense driver -- baked in from prebuilt ROS debs
# (ros-humble-realsense2-camera pulls in ros-humble-librealsense2 with it)
RUN apt-get install -y --no-install-recommends \
        ros-humble-librealsense2 \
        ros-humble-realsense2-camera \
        ros-humble-realsense2-description \
        ros-humble-image-transport \
        ros-humble-image-transport-plugins \
        && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc

# range_libc with real GPU acceleration
# - cython must be <3.0 (range_libc's .pyx predates Cython 3 and won't compile with it)
# - upstream setup.py hardcodes -arch=sm_20, which CUDA 12 (JetPack 6) removed
#   entirely, so we patch it to the target GPU arch before building
# - kept ABOVE the src/ COPY so editing your code never triggers a rebuild of this

RUN pip3 install "cython" transforms3d

RUN mkdir -p /tmp/build && \
    cd /tmp/build && \
    git clone https://github.com/f1tenth/range_libc.git && \
    cd range_libc/pywrapper && \
    sed -i 's|compiler_flags = \["-w","-std=c++11"|compiler_flags = ["-w","-std=c++11","-fPIC"|' setup.py && \
    sed -i 's|-arch=sm_62|-arch=sm_87|' setup.py && \
    sed -i '/-DCMAKE_C_COMPILER/d; /-DCMAKE_CXX_COMPILER/d' setup.py && \
    sed -i "s|postargs = extra_postargs\['gcc'\]|self.set_executable('compiler_so', default_compiler_so); postargs = extra_postargs['gcc']|" setup.py && \
    WITH_CUDA=ON python3 setup.py install && \
    /bin/bash -c "source /opt/ros/humble/setup.bash && python3 -c \"import range_libc; print('range_libc OK')\"" && \
    cd / && rm -rf /tmp/build

# JAX with CUDA for the MPPI controller (mppi_example)
# - copied from jax_src (dustynv/jax:r36.4.0) instead of pip: no live index
#   serves a Jetson/Orin-verified build (see FROM jax_src note up top)
# - numpy/scipy/ml_dtypes/opt_einsum pulled from the same image so the whole
#   set is the exact combination that was verified working, not a mix-and-match
#ARG PYDIST=/usr/local/lib/python3.10/dist-packages
#COPY --from=jax_src ${PYDIST}/numpy ${PYDIST}/numpy
#COPY --from=jax_src ${PYDIST}/numpy.libs ${PYDIST}/numpy.libs
#COPY --from=jax_src ${PYDIST}/numpy-1.26.4.dist-info ${PYDIST}/numpy-1.26.4.dist-info
#COPY --from=jax_src ${PYDIST}/scipy ${PYDIST}/scipy
#COPY --from=jax_src ${PYDIST}/scipy.libs ${PYDIST}/scipy.libs
#COPY --from=jax_src ${PYDIST}/scipy-1.14.1.dist-info ${PYDIST}/scipy-1.14.1.dist-info
#COPY --from=jax_src ${PYDIST}/ml_dtypes ${PYDIST}/ml_dtypes
#COPY --from=jax_src ${PYDIST}/ml_dtypes-0.5.0.dist-info ${PYDIST}/ml_dtypes-0.5.0.dist-info
#COPY --from=jax_src ${PYDIST}/opt_einsum ${PYDIST}/opt_einsum
#COPY --from=jax_src ${PYDIST}/opt_einsum-3.4.0.dist-info ${PYDIST}/opt_einsum-3.4.0.dist-info
#COPY --from=jax_src ${PYDIST}/jax ${PYDIST}/jax
#COPY --from=jax_src "${PYDIST}/jax-0.4.35.dev20241015+b076890.dist-info" "${PYDIST}/jax-0.4.35.dev20241015+b076890.dist-info"
#COPY --from=jax_src ${PYDIST}/jaxlib ${PYDIST}/jaxlib
#COPY --from=jax_src ${PYDIST}/jaxlib-0.4.35.dev20241015.dist-info ${PYDIST}/jaxlib-0.4.35.dev20241015.dist-info
#COPY --from=jax_src ${PYDIST}/jax_cuda12_plugin ${PYDIST}/jax_cuda12_plugin
#COPY --from=jax_src ${PYDIST}/jax_cuda12_plugin-0.4.35.dev20241015.dist-info ${PYDIST}/jax_cuda12_plugin-0.4.35.dev20241015.dist-info
#COPY --from=jax_src ${PYDIST}/jax_plugins ${PYDIST}/jax_plugins
#COPY --from=jax_src ${PYDIST}/jax_cuda12_pjrt-0.4.35.dev20241015.dist-info ${PYDIST}/jax_cuda12_pjrt-0.4.35.dev20241015.dist-info

# cuDNN 9.4.0 (jax_src's system cuDNN, what jaxlib was actually linked/compiled
# against) replacing this base image's 9.3.0 -- jaxlib refuses to init on an
# older-minor-version runtime cuDNN ("Loaded runtime CuDNN library: 9.3.0 but
# source was compiled with: 9.4.0"), found by actually running a GPU op, not
# just jax.devices(). Copying both the .so.9 symlinks and their .so.9.4.0
# targets together so the symlinks resolve correctly in this image.
#ARG CUDNN_LIB=/usr/lib/aarch64-linux-gnu
#COPY --from=jax_src ${CUDNN_LIB}/libcudnn.so.9.4.0 ${CUDNN_LIB}/libcudnn.so.9.4.0
#COPY --from=jax_src ${CUDNN_LIB}/libcudnn.so.9 ${CUDNN_LIB}/libcudnn.so.9
#COPY --from=jax_src ${CUDNN_LIB}/libcudnn_adv.so.9.4.0 ${CUDNN_LIB}/libcudnn_adv.so.9.4.0
#COPY --from=jax_src ${CUDNN_LIB}/libcudnn_adv.so.9 ${CUDNN_LIB}/libcudnn_adv.so.9
#COPY --from=jax_src ${CUDNN_LIB}/libcudnn_cnn.so.9.4.0 ${CUDNN_LIB}/libcudnn_cnn.so.9.4.0
#COPY --from=jax_src ${CUDNN_LIB}/libcudnn_cnn.so.9 ${CUDNN_LIB}/libcudnn_cnn.so.9
#COPY --from=jax_src ${CUDNN_LIB}/libcudnn_engines_precompiled.so.9.4.0 ${CUDNN_LIB}/libcudnn_engines_precompiled.so.9.4.0
#COPY --from=jax_src ${CUDNN_LIB}/libcudnn_engines_precompiled.so.9 ${CUDNN_LIB}/libcudnn_engines_precompiled.so.9
#COPY --from=jax_src ${CUDNN_LIB}/libcudnn_engines_runtime_compiled.so.9.4.0 ${CUDNN_LIB}/libcudnn_engines_runtime_compiled.so.9.4.0
#COPY --from=jax_src ${CUDNN_LIB}/libcudnn_engines_runtime_compiled.so.9 ${CUDNN_LIB}/libcudnn_engines_runtime_compiled.so.9
#COPY --from=jax_src ${CUDNN_LIB}/libcudnn_graph.so.9.4.0 ${CUDNN_LIB}/libcudnn_graph.so.9.4.0
#COPY --from=jax_src ${CUDNN_LIB}/libcudnn_graph.so.9 ${CUDNN_LIB}/libcudnn_graph.so.9
#COPY --from=jax_src ${CUDNN_LIB}/libcudnn_heuristic.so.9.4.0 ${CUDNN_LIB}/libcudnn_heuristic.so.9.4.0
#COPY --from=jax_src ${CUDNN_LIB}/libcudnn_heuristic.so.9 ${CUDNN_LIB}/libcudnn_heuristic.so.9
#COPY --from=jax_src ${CUDNN_LIB}/libcudnn_ops.so.9.4.0 ${CUDNN_LIB}/libcudnn_ops.so.9.4.0
#COPY --from=jax_src ${CUDNN_LIB}/libcudnn_ops.so.9 ${CUDNN_LIB}/libcudnn_ops.so.9

# Import-only check: docker build steps don't reliably get GPU device access
# the way `docker run --runtime nvidia` does, so jax.devices() is verified at
# container runtime instead, not here.
#RUN python3 -c "import jax; print('jax import OK:', jax.__version__)"

# copy in the code
RUN mkdir -p /home/bolty/ament_ws
WORKDIR /home/bolty/ament_ws
COPY src/ src

# use rosdep to solve the remaining dependencies
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && apt-get update && \
    rosdep install -r --from-paths src --ignore-src --rosdistro humble -y" && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# rosdep pulls in apt's python3-numba (0.55.1), which predates support for
# the numpy 1.26.4 copied in from jax_src above -- importing it crashes with
# "SystemError: initialization of _internal failed without raising an
# exception" (numba's compiled _internal extension doesn't recognize this
# numpy ABI). Pin a numba that supports numpy 1.26 via pip instead; it lands
# in /usr/local/lib/python3.10/dist-packages, which sys.path resolves before
# apt's /usr/lib/python3/dist-packages, so it shadows the broken one without
# needing to uninstall it.
#RUN pip3 install "numba==0.59.1" "numpy==1.26.4" && \ python3 -c "from numba import njit; print('numba OK')"

# build the ros 2 project
#RUN . /opt/ros/humble/setup.sh && \
    #colcon build

# clean up the directory
# NOTE: kept from the original -- but be aware this also wipes install/,
# i.e. the colcon build above does not persist into the final image.
# Remove install/* from this line if you want the built workspace baked in.
#RUN rm -rf src/* build/* devel/* install/* log/*

# entry point
COPY docker/wato_ros_entrypoint.sh /home/bolty/ament_ws/wato_ros_entrypoint.sh
RUN chmod +x /home/bolty/ament_ws/wato_ros_entrypoint.sh
ENTRYPOINT ["/home/bolty/ament_ws/wato_ros_entrypoint.sh"]
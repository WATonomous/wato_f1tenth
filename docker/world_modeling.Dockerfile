# Built locally with `./watod build-base` until prebuilt images are published.
# The base is picked by architecture and GPU (`GPU` from watod: nvidia or none). Only the
# Jetson uses the GPU here (range_libc's rmgpu); x86 PCs run the particle filter on CPU.
ARG GPU=nvidia
ARG BASE_IMAGE_CPU=ghcr.io/watonomous/wato_f1tenth/base:jazzy-ubuntu24.04
ARG BASE_IMAGE_ARM64_NVIDIA=ghcr.io/watonomous/wato_f1tenth/base:jetson-cuda12.8-trt-jazzy-ubuntu24.04

FROM ${BASE_IMAGE_ARM64_NVIDIA} AS base_arm64_nvidia
FROM ${BASE_IMAGE_CPU} AS base_arm64_none
FROM ${BASE_IMAGE_CPU} AS base_amd64_nvidia
FROM ${BASE_IMAGE_CPU} AS base_amd64_none

################################ Source ################################
# NOTE: You should add in the source stage in the following order:
#   - clone git repositories -> copy source code
# This will make your builds significantly faster
FROM base_${TARGETARCH}_${GPU} AS source

WORKDIR ${AMENT_WS}/src

# Clone external repositories here, e.g.
# RUN git clone https://github.com/WATonomous/<repo>.git <repo>

# Copy in source code
COPY src/world_modeling world_modeling
COPY src/infrastructure/wato_lifecycle_manager wato_lifecycle_manager
COPY src/wato_test wato_test

################################# Dependencies ################################
# NOTE: You should be relying on ROSDEP as much as possible
# Use this stage as a last resort
FROM base_${TARGETARCH}_${GPU} AS dependencies
ARG TARGETARCH
ARG GPU

# range_libc: ray casting for particle_filter (not a ROS package, so not rosdep).
# Same patches the mppi_test robot image used, plus:
#   - Cython < 3: the .pyx files predate Cython 3
#   - built through pip so setuptools' distutils shim works on Python 3.12
#   - no -march=native, so an image built on one machine runs on others
#   - CUDA (rmgpu, sm_87) only on the Jetson image (arm64 + GPU=nvidia); otherwise CPU methods
ARG RANGE_LIBC_COMMIT=781acfc9edea977b564c4cb5bdaa86e8f719ab7b
# hadolint ignore=DL3003
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-dev python3-numpy python3-setuptools python3-transforms3d && \
    rm -rf /var/lib/apt/lists/* && \
    pip3 install --no-cache-dir "cython<3" && \
    git clone https://github.com/f1tenth/range_libc.git /tmp/range_libc && \
    cd /tmp/range_libc && git checkout "${RANGE_LIBC_COMMIT}" && cd pywrapper && \
    sed -i 's|compiler_flags = \["-w","-std=c++11"|compiler_flags = ["-w","-std=c++11","-fPIC"|' setup.py && \
    sed -i 's|, "-march=native"||' setup.py && \
    sed -i 's|-arch=sm_62|-arch=sm_87|' setup.py && \
    sed -i '/-DCMAKE_C_COMPILER/d; /-DCMAKE_CXX_COMPILER/d' setup.py && \
    sed -i "s|postargs = extra_postargs\['gcc'\]|self.set_executable('compiler_so', default_compiler_so); postargs = extra_postargs['gcc']|" setup.py && \
    if [ "${TARGETARCH}" = "arm64" ] && [ "${GPU}" = "nvidia" ]; then export WITH_CUDA=ON; else export WITH_CUDA=OFF; fi && \
    pip3 install --no-cache-dir --no-build-isolation . && \
    bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && python3 -c \"import range_libc; print('range_libc OK, CUDA: ${WITH_CUDA}')\"" && \
    rm -rf /tmp/range_libc

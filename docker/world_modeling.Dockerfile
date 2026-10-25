# Built locally with `./watod build-base` until prebuilt images are published.
# The base is picked per architecture: x86 PCs and the Jetson (arm64) need different CUDA stacks.
ARG BASE_IMAGE_AMD64=ghcr.io/watonomous/wato_f1tenth/base:jazzy-ubuntu24.04
ARG BASE_IMAGE_ARM64=ghcr.io/watonomous/wato_f1tenth/base:jetson-cuda12.8-trt-jazzy-ubuntu24.04

FROM ${BASE_IMAGE_AMD64} AS base_amd64
FROM ${BASE_IMAGE_ARM64} AS base_arm64

################################ Source ################################
# NOTE: You should add in the source stage in the following order:
#   - clone git repositories -> copy source code
# This will make your builds significantly faster
FROM base_${TARGETARCH} AS source

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
FROM base_${TARGETARCH} AS dependencies
ARG TARGETARCH

# range_libc: ray casting for particle_filter (not a ROS package, so not rosdep).
# Same patches the mppi_test robot image used, plus:
#   - Cython < 3: the .pyx files predate Cython 3
#   - built through pip so setuptools' distutils shim works on Python 3.12
#   - no -march=native, so an image built on one machine runs on others
#   - CUDA (rmgpu, sm_87) only on the Jetson image; x86 builds the CPU methods
ARG RANGE_LIBC_COMMIT=781acfc9edea977b564c4cb5bdaa86e8f719ab7b
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-dev python3-numpy python3-setuptools && \
    rm -rf /var/lib/apt/lists/* && \
    pip3 install --no-cache-dir "cython<3" && \
    git clone https://github.com/f1tenth/range_libc.git /tmp/range_libc && \
    cd /tmp/range_libc && git checkout "${RANGE_LIBC_COMMIT}" && cd pywrapper && \
    sed -i 's|compiler_flags = \["-w","-std=c++11"|compiler_flags = ["-w","-std=c++11","-fPIC"|' setup.py && \
    sed -i 's|, "-march=native"||' setup.py && \
    sed -i 's|-arch=sm_62|-arch=sm_87|' setup.py && \
    sed -i '/-DCMAKE_C_COMPILER/d; /-DCMAKE_CXX_COMPILER/d' setup.py && \
    sed -i "s|postargs = extra_postargs\['gcc'\]|self.set_executable('compiler_so', default_compiler_so); postargs = extra_postargs['gcc']|" setup.py && \
    if [ "${TARGETARCH}" = "arm64" ]; then export WITH_CUDA=ON; else export WITH_CUDA=OFF; fi && \
    pip3 install --no-cache-dir --no-build-isolation . && \
    python3 -c "import range_libc; print('range_libc OK, CUDA:', '${WITH_CUDA}')" && \
    rm -rf /tmp/range_libc

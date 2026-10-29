ARG BASE_IMAGE=ghcr.io/watonomous/wato_f1tenth/base:jazzy-ubuntu24.04
# Built locally with `./watod build-base` until prebuilt images are published.

################################ Source ################################
# NOTE: You should add in the source stage in the following order:
#   - clone git repositories -> copy source code
# This will make your builds significantly faster
FROM ${BASE_IMAGE} AS source

WORKDIR ${AMENT_WS}/src

# Clone external repositories here, e.g.
# RUN git clone https://github.com/WATonomous/<repo>.git <repo>

# Copy in source code
COPY src/interfacing interfacing
# ControlStatus for drive_status
COPY src/infrastructure/f1tenth_msgs f1tenth_msgs
COPY src/infrastructure/wato_lifecycle_manager wato_lifecycle_manager
COPY src/wato_test wato_test

################################# Dependencies ################################
# NOTE: You should be relying on ROSDEP as much as possible
# Use this stage as a last resort
FROM ${BASE_IMAGE} AS dependencies

ARG TARGETARCH

# RealSense on arm64: the apt librealsense uses the kernel backend, which reads the D435i
# IMU through the hid_sensor IIO drivers. JetPack kernels don't ship those, so the IMU
# fails ("HID Motion Sensor Failure"). Rebuild the same version with the RSUSB (libusb)
# backend over the apt library (same SONAME, so the apt realsense2_camera loads it) and
# hold the package so the rosdep stage keeps it. x86 keeps the apt build.
ARG LIBREALSENSE_VERSION=2.58.4
RUN if [ "${TARGETARCH}" = "arm64" ]; then \
      apt-get update && \
      apt-get install -y --no-install-recommends \
        "ros-${ROS_DISTRO}-librealsense2=${LIBREALSENSE_VERSION}-*" "ros-${ROS_DISTRO}-realsense2-camera" \
        git cmake build-essential pkg-config libusb-1.0-0-dev libudev-dev libssl-dev && \
      apt-mark hold "ros-${ROS_DISTRO}-librealsense2" && \
      git clone --depth 1 --branch "v${LIBREALSENSE_VERSION}" https://github.com/IntelRealSense/librealsense.git /tmp/librealsense && \
      cmake -S /tmp/librealsense -B /tmp/librealsense/build \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX="/opt/ros/${ROS_DISTRO}" \
        -DCMAKE_INSTALL_LIBDIR=lib/aarch64-linux-gnu \
        -DFORCE_RSUSB_BACKEND=ON \
        -DBUILD_EXAMPLES=OFF -DBUILD_GRAPHICAL_EXAMPLES=OFF -DBUILD_TOOLS=OFF \
        -DBUILD_GLSL_EXTENSIONS=OFF -DBUILD_PYTHON_BINDINGS=OFF -DBUILD_UNIT_TESTS=OFF \
        -DBUILD_WITH_CUDA=OFF -DCHECK_FOR_UPDATES=OFF -DBUILD_NETWORK_DEVICE=OFF && \
      cmake --build /tmp/librealsense/build -j4 && \
      cmake --install /tmp/librealsense/build && \
      rm -rf /tmp/librealsense /var/lib/apt/lists/*; \
    fi

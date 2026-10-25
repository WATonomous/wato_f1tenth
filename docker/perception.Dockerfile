# Built locally with `./watod build-base` until prebuilt images are published.
# The base is picked per architecture: x86 PCs and the Jetson (arm64) need different CUDA stacks.
ARG BASE_IMAGE_AMD64=ghcr.io/watonomous/wato_f1tenth/base:cuda12.9-trt-jazzy-ubuntu24.04
ARG BASE_IMAGE_ARM64=ghcr.io/watonomous/wato_f1tenth/base:jetson-cuda12.8-trt-jazzy-ubuntu24.04

FROM ${BASE_IMAGE_AMD64} AS base_amd64
FROM ${BASE_IMAGE_ARM64} AS base_arm64

################################ Source ################################
# NOTE: You should add in the source stage in the following order:
#   - clone git repositories -> copy source code
# This will make your builds significantly faster
FROM base_${TARGETARCH} AS source

WORKDIR ${AMENT_WS}/src

# Clone external repositories here, e.g. WATonomous/deep_ros for ONNX/TensorRT inference:
# RUN git clone https://github.com/WATonomous/deep_ros.git deep_ros

# Copy in source code
COPY src/perception perception
COPY src/infrastructure/wato_lifecycle_manager wato_lifecycle_manager
COPY src/wato_test wato_test

################################# Dependencies ################################
# NOTE: You should be relying on ROSDEP as much as possible
# Use this stage as a last resort
FROM base_${TARGETARCH} AS dependencies

# Use bash with pipefail so piped RUN commands (e.g. curl | gpg) fail loudly.
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Install GPU / inference dependencies (non-rosdep) here, e.g. TensorRT:
# RUN apt-get update && \
#     apt-get install -y --no-install-recommends \
#     libnvinfer10 \
#     libnvinfer-plugin10 \
#     libnvonnxparsers10 && \
#     rm -rf /var/lib/apt/lists/*

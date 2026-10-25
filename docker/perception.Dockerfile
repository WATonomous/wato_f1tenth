ARG BASE_IMAGE=ghcr.io/watonomous/wato_monorepo/base:cuda12.8.1-cudnn-runtime-ubuntu24.04
# Base images are published by wato_monorepo (.github/workflows/build_base_images.yml).
# To host your own, run that workflow in this repo and point BASE_IMAGE at your registry.

################################ Source ################################
# NOTE: You should add in the source stage in the following order:
#   - clone git repositories -> copy source code
# This will make your builds significantly faster
FROM ${BASE_IMAGE} AS source

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
FROM ${BASE_IMAGE} AS dependencies

# Use bash with pipefail so piped RUN commands (e.g. curl | gpg) fail loudly.
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Install GPU / inference dependencies (non-rosdep) here, e.g. TensorRT:
# RUN apt-get update && \
#     apt-get install -y --no-install-recommends \
#     libnvinfer10 \
#     libnvinfer-plugin10 \
#     libnvonnxparsers10 && \
#     rm -rf /var/lib/apt/lists/*

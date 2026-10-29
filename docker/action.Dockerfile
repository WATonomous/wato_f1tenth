# Built locally with `./watod build-base` until prebuilt images are published.
# The base is picked by architecture and GPU (`GPU` from watod: nvidia or none):
#   amd64 + nvidia -> CUDA 12.9 + TensorRT   amd64 + none -> CPU only
#   arm64 + nvidia -> Jetson CUDA + TensorRT  arm64 + none -> CPU only
ARG GPU=nvidia
ARG BASE_IMAGE_CPU=ghcr.io/watonomous/wato_f1tenth/base:jazzy-ubuntu24.04
ARG BASE_IMAGE_AMD64_NVIDIA=ghcr.io/watonomous/wato_f1tenth/base:cuda12.9-trt-jazzy-ubuntu24.04
ARG BASE_IMAGE_ARM64_NVIDIA=ghcr.io/watonomous/wato_f1tenth/base:jetson-cuda12.8-trt-jazzy-ubuntu24.04

FROM ${BASE_IMAGE_AMD64_NVIDIA} AS base_amd64_nvidia
FROM ${BASE_IMAGE_ARM64_NVIDIA} AS base_arm64_nvidia
FROM ${BASE_IMAGE_CPU} AS base_amd64_none
FROM ${BASE_IMAGE_CPU} AS base_arm64_none

################################ Source ################################
# NOTE: You should add in the source stage in the following order:
#   - clone git repositories -> copy source code
# This will make your builds significantly faster
FROM base_${TARGETARCH}_${GPU} AS source

WORKDIR ${AMENT_WS}/src

# Clone external repositories here, e.g.
# RUN git clone https://github.com/WATonomous/<repo>.git <repo>

# Copy in source code
COPY src/action action
COPY src/wato_msgs wato_msgs
# track maps and racelines (MPPI's raceline and wall cost)
COPY src/world_modeling/race_tracks race_tracks
COPY src/infrastructure/wato_lifecycle_manager wato_lifecycle_manager
COPY src/wato_test wato_test

################################# Dependencies ################################
# NOTE: You should be relying on ROSDEP as much as possible
# Use this stage as a last resort
FROM base_${TARGETARCH}_${GPU} AS dependencies

# Install module-specific dependencies (non-rosdep) here, e.g.
# RUN apt-get update && apt-get install -y --no-install-recommends <pkg> && \
#     rm -rf /var/lib/apt/lists/*

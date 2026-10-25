ARG BASE_IMAGE=ghcr.io/watonomous/wato_monorepo/base:jazzy-ubuntu24.04
# Base images are published by wato_monorepo (.github/workflows/build_base_images.yml).
# To host your own, run that workflow in this repo and point BASE_IMAGE at your registry.

################################ Source ################################
# NOTE: You should add in the source stage in the following order:
#   - clone git repositories -> copy source code
# This will make your builds significantly faster
FROM ${BASE_IMAGE} AS source

WORKDIR ${AMENT_WS}/src

# Clone external repositories here, e.g.
# RUN git clone https://github.com/WATonomous/<repo>.git <repo>

# Copy in source code
COPY src/action action
COPY src/infrastructure/wato_lifecycle_manager wato_lifecycle_manager
COPY src/wato_test wato_test

################################# Dependencies ################################
# NOTE: You should be relying on ROSDEP as much as possible
# Use this stage as a last resort
FROM ${BASE_IMAGE} AS dependencies

# Install module-specific dependencies (non-rosdep) here, e.g.
# RUN apt-get update && apt-get install -y --no-install-recommends <pkg> && \
#     rm -rf /var/lib/apt/lists/*

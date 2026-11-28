ARG MODULE_SOURCE=interfacing:source
ARG MODULE_DEPS=interfacing:deps

################################ Rosdep Scan ################################
# Reference the 'source' stage from the module image
FROM ${MODULE_SOURCE} AS rosdep_scan
ARG MODULE_SOURCE
ARG MODULE_DEPS

# Scan for rosdeps from the source stage
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
# rosdep -s resolves keys from its own database and checks what is installed with dpkg, so it
# needs no apt index: no apt-get update here, which ran on every source change (same output on
# every module, checked 2026-10-29)
RUN rosdep install --from-paths . --ignore-src -r -s > /tmp/rosdep_output && \
    (grep 'apt-get install' /tmp/rosdep_output || true) \
        | awk '{print $3}' \
        | sort > /tmp/colcon_install_list && \
    (grep 'pip3 install' /tmp/rosdep_output || true) \
        | sed 's/.*pip3 install //' \
        | sed 's/ --hash=[^ ]*//g' \
        | sort > /tmp/colcon_pip_install_list

################################ Install Rosdeps ################################
# Use the dependencies stage from the module image
FROM ${MODULE_DEPS} AS rosdep_install
ARG MODULE_SOURCE
ARG MODULE_DEPS

# Install Rosdep requirements
COPY --from=rosdep_scan /tmp/colcon_install_list /tmp/colcon_install_list
COPY --from=rosdep_scan /tmp/colcon_pip_install_list /tmp/colcon_pip_install_list
RUN apt-get update && \
    apt-get install -qq -y --no-install-recommends ccache && \
    if [ -s /tmp/colcon_install_list ]; then \
      xargs -a /tmp/colcon_install_list apt-fast install -qq -y --no-install-recommends; \
    fi && \
    rm -rf /var/lib/apt/lists/* && \
    if [ -s /tmp/colcon_pip_install_list ]; then \
      xargs -a /tmp/colcon_pip_install_list pip3 install --no-cache-dir; \
    fi

# Dependency Cleanup
WORKDIR /
RUN apt-get -qq autoremove -y && apt-get -qq autoclean && apt-get -qq clean && \
    rm -rf /root/* /root/.ros /tmp/* /var/lib/apt/lists/* /usr/share/doc/*

# RMW Configurations
COPY docker/config/rmw_zenoh_router_config.json5 ${WATONOMOUS_INSTALL}/rmw_zenoh_router_config.json5
COPY docker/config/rmw_zenoh_session_config.json5 ${WATONOMOUS_INSTALL}/rmw_zenoh_session_config.json5

# Entrypoint
COPY docker/config/wato_entrypoint.sh ${WATONOMOUS_INSTALL}/wato_entrypoint.sh
ENTRYPOINT ["/opt/watonomous/wato_entrypoint.sh"]

################################ Build ################################
FROM rosdep_install AS build

# Copy in source code from rosdep_scan stage (which comes from MODULE_SOURCE). Kept out of
# rosdep_install so `develop` (which bind-mounts source from the host) never rebuilds on code edits,
# and kept last before colcon build so a source change skips dependency install and cleanup.
WORKDIR ${AMENT_WS}
COPY --from=rosdep_scan ${AMENT_WS}/src src

# Build and Install ROS2 packages. ccache keeps compiled objects in a BuildKit cache mount shared by
# every module and kept between image builds, so a rebuild only compiles files that changed, and
# packages copied into several modules (wato_test, wato_lifecycle_manager) compile once. The cache
# lives in Docker's build cache, not in the image (`docker builder prune` clears it). build/ stays
# in the image: the test images (this stage) run colcon test from it.
ARG TARGETARCH
# content: hash the compiler itself, so objects are shared between modules on different base images
ENV CCACHE_DIR=/ccache CCACHE_COMPILERCHECK=content CCACHE_MAXSIZE=5G
RUN --mount=type=cache,id=f1tenth-ccache-${TARGETARCH},target=/ccache \
    . "/opt/ros/${ROS_DISTRO}/setup.sh" && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache \
      -DCMAKE_CUDA_COMPILER_LAUNCHER=ccache && \
    (ccache --show-stats || true) && \
    cp -r install/. "${WATONOMOUS_INSTALL}"

################################ Deploy ################################
FROM build AS deploy

# Source Cleanup, Security Setup, and Workspace Setup
RUN rm -rf "${AMENT_WS:?}"/*

# Stamp git metadata last (metadata-only layer) so floating local tags can warn on branch mismatch
# without invalidating the build/cleanup layers above.
ARG GIT_BRANCH=unknown
ARG GIT_COMMIT=unknown
ARG MONO_DIR=unknown
LABEL org.watonomous.git_branch="${GIT_BRANCH}" \
      org.watonomous.git_commit="${GIT_COMMIT}" \
      org.watonomous.mono_dir="${MONO_DIR}"

################################ Develop ################################
FROM rosdep_install AS develop
ARG USERNAME
ARG USER_GID
ARG USER_UID
ARG CLAUDE_CODE

# Update Sources and Install Useful Developer Tools
# hadolint ignore=DL3009
RUN apt-get update && \
    apt-fast install -qq -y --no-install-recommends \
    tmux \
    git \
    curl \
    wget \
    htop \
    nano \
    tree \
    can-utils

# Set user in container to developer's user
# hadolint ignore=SC2086
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN existing_user=$(getent passwd ${USER_UID} | cut -d: -f1 || true) \
    && if [ -n "$existing_user" ]; then userdel -r "$existing_user" 2>/dev/null || true; fi \
    && if ! getent group ${USER_GID} >/dev/null; then groupadd --gid ${USER_GID} ${USERNAME}; fi \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m $USERNAME --shell /bin/bash \
    && apt-get update \
    && apt-get install -y --no-install-recommends sudo \
    && echo $USERNAME ALL=\(ALL\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && cp /etc/skel/.bashrc /home/$USERNAME/.bashrc \
    && cp /etc/skel/.profile /home/$USERNAME/.profile \
    && chown $USERNAME:$USERNAME /home/$USERNAME/.bashrc /home/$USERNAME/.profile \
    && mkdir -p "${AMENT_WS}/src" \
    && chown -R "${USERNAME}":"${USERNAME}" "${AMENT_WS}" \
    && rm -rf /var/lib/apt/lists/*

USER $USERNAME

# Optionally install Claude Code
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN if [ "$CLAUDE_CODE" = "true" ]; then \
      curl -fsSL https://claude.ai/install.sh | bash; \
    fi

# Setup dev bashrc
COPY docker/config/wato_dev.bashrc ${WATONOMOUS_INSTALL}/wato_dev.bashrc
RUN echo "source ${WATONOMOUS_INSTALL}/wato_dev.bashrc" >> ~/.bashrc

# Default to opening in the AMENT_WS
WORKDIR ${AMENT_WS}

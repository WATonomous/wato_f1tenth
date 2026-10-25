ARG GENERIC_IMAGE=ubuntu:24.04

################ CUDA, cuDNN and TensorRT for Jetson Orin on JetPack 6 ################
# Ubuntu 24.04 user space on a JetPack 6 (L4T r36) host. JetPack 6 can run a newer CUDA
# stack than it ships with ("upgradable compute stack"); cuda-compat carries the
# matching user-mode driver pieces. Package choices follow jetson-containers'
# JetPack 6 + CUDA 12.8 recipe (packages/cuda/cuda and packages/cuda/cudastack),
# installed directly from NVIDIA rather than using its images.
#
# Build this on the Jetson (arm64). TensorRT engines are tied to the TensorRT version
# and GPU: rebuild them on the car when these versions change.
FROM ${GENERIC_IMAGE} AS jetson_cuda

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG DEBIAN_FRONTEND=noninteractive

ARG CUDA_VERSION=12.8
ARG CUDA_URL=https://developer.download.nvidia.com/compute/cuda/12.8.1/local_installers/cuda-tegra-repo-ubuntu2204-12-8-local_12.8.1-1_arm64.deb
ARG CUDA_PIN_URL=https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/arm64/cuda-ubuntu2204.pin
ARG CUDA_PACKAGES="cuda-toolkit-12-8"
ARG CUDNN_URL=https://developer.download.nvidia.com/compute/cudnn/9.8.0/local_installers/cudnn-local-tegra-repo-ubuntu2404-9.8.0_1.0-1_arm64.deb
ARG CUDNN_PACKAGES="libcudnn9-cuda-12 libcudnn9-dev-cuda-12"
ARG TENSORRT_URL=https://developer.nvidia.com/downloads/compute/machine-learning/tensorrt/10.7.0/tars/TensorRT-10.7.0.23.l4t.aarch64-gnu.cuda-12.6.tar.gz

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        binutils ca-certificates wget xz-utils \
        libegl1 libgles2 libglvnd0 libgl1 && \
    rm -rf /var/lib/apt/lists/*

# CUDA toolkit (nvcc included, so CUDA code builds in the image) + cuda-compat
RUN mkdir -p /tmp/cuda && cd /tmp/cuda && \
    wget -q "${CUDA_PIN_URL}" -O /etc/apt/preferences.d/cuda-repository-pin-600 && \
    wget -q "${CUDA_URL}" && \
    dpkg -i ./*.deb && \
    cp /var/cuda-*-local/cuda-*-keyring.gpg /usr/share/keyrings/ && \
    if compgen -G "/var/cuda-tegra-repo-ubuntu*-local/cuda-compat-*.deb" > /dev/null; then \
        ar x /var/cuda-tegra-repo-ubuntu*-local/cuda-compat-*.deb && tar xf data.tar.xz -C / ; \
    fi && \
    apt-get update && \
    apt-get install -y --no-install-recommends ${CUDA_PACKAGES} && \
    dpkg -P "$(basename "${CUDA_URL}" | cut -d_ -f1)" && \
    rm -rf /var/lib/apt/lists/* /tmp/cuda

# cuDNN
RUN mkdir -p /tmp/cudnn && cd /tmp/cudnn && \
    wget -q "${CUDNN_URL}" && \
    dpkg -i ./*.deb && \
    cp /var/cudnn-*-repo-*/cudnn-*-keyring.gpg /usr/share/keyrings/ && \
    apt-get update && \
    apt-get install -y --no-install-recommends ${CUDNN_PACKAGES} && \
    dpkg -P "$(basename "${CUDNN_URL}" | cut -d_ -f1)" && \
    rm -rf /var/lib/apt/lists/* /tmp/cudnn

# TensorRT (L4T tarball: libraries, headers, trtexec)
RUN mkdir -p /tmp/trt && cd /tmp/trt && \
    wget -q -O trt.tar.gz "${TENSORRT_URL}" && \
    tar -xzf trt.tar.gz --strip-components=1 && \
    cp -r lib/*.so* /usr/local/cuda/targets/aarch64-linux/lib/ && \
    cp -r include/* /usr/include/ && \
    cp bin/trtexec /usr/local/bin/ && \
    echo "/usr/local/cuda/targets/aarch64-linux/lib" > /etc/ld.so.conf.d/tensorrt.conf && \
    ldconfig && \
    rm -rf /tmp/trt

# Jetson GPU access from containers: Tegra libraries are mounted by the NVIDIA container
# runtime; point the loader and EGL at them (same as the orin branch's robot image).
RUN echo "/usr/lib/aarch64-linux-gnu/tegra" >> /etc/ld.so.conf.d/nvidia-tegra.conf && \
    echo "/usr/lib/aarch64-linux-gnu/tegra-egl" >> /etc/ld.so.conf.d/nvidia-tegra.conf && \
    rm -rf /usr/share/glvnd/egl_vendor.d && mkdir -p /usr/share/glvnd/egl_vendor.d && \
    echo '{"file_format_version": "1.0.0", "ICD": {"library_path": "libEGL_nvidia.so.0"}}' \
        > /usr/share/glvnd/egl_vendor.d/10_nvidia.json

ENV CUDA_HOME=/usr/local/cuda \
    PATH=/usr/local/cuda/bin:${PATH} \
    LD_LIBRARY_PATH=/usr/local/cuda/compat:/usr/local/cuda/lib64 \
    CUDAARCHS=87 \
    CUDA_ARCHITECTURES=87 \
    NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=all

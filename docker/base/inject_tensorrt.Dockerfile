ARG GENERIC_IMAGE

######################## TensorRT for x86 PCs with an NVIDIA GPU ########################
# Applied on top of nvcr.io/nvidia/cuda:*-cudnn-devel-ubuntu24.04, which already has
# NVIDIA's CUDA apt repository configured. NVIDIA's Ubuntu 24.04 x86 TensorRT packages
# are built against CUDA 12.9, so the PC image uses CUDA 12.9 (the Jetson uses 12.8;
# engines are rebuilt per target anyway).
FROM ${GENERIC_IMAGE} AS tensorrt

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG DEBIAN_FRONTEND=noninteractive
ARG TENSORRT_VERSION=10.16.1.11-1+cuda12.9

# Pin every TensorRT package (including the ones pulled in as dependencies) to the
# CUDA 12.9 build; without the pin apt picks the newest +cuda13.x builds for them.
# Runtime libraries + headers + trtexec only: the -dev packages are ~7 GB of static
# libraries we never link.
RUN printf 'Package: libnvinfer* libnvonnxparsers* python3-libnvinfer* tensorrt*\nPin: version %s\nPin-Priority: 1001\n' \
        "${TENSORRT_VERSION}" > /etc/apt/preferences.d/tensorrt && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        libnvinfer10 libnvinfer-plugin10 libnvonnxparsers10 \
        libnvinfer-headers-dev libnvinfer-headers-plugin-dev libnvinfer-bin && \
    rm -rf /var/lib/apt/lists/*

# Plain .so links so CMake can link against the shared libraries without the -dev packages.
RUN cd /usr/lib/x86_64-linux-gnu && \
    ln -sf libnvinfer.so.10 libnvinfer.so && \
    ln -sf libnvinfer_plugin.so.10 libnvinfer_plugin.so && \
    ln -sf libnvonnxparser.so.10 libnvonnxparser.so

ENV PATH=/usr/src/tensorrt/bin:${PATH}

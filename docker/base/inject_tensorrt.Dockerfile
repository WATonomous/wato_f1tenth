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

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        "libnvinfer10=${TENSORRT_VERSION}" \
        "libnvinfer-plugin10=${TENSORRT_VERSION}" \
        "libnvonnxparsers10=${TENSORRT_VERSION}" \
        "libnvinfer-dev=${TENSORRT_VERSION}" \
        "libnvinfer-headers-dev=${TENSORRT_VERSION}" \
        "libnvinfer-plugin-dev=${TENSORRT_VERSION}" \
        "libnvinfer-headers-plugin-dev=${TENSORRT_VERSION}" \
        "libnvonnxparsers-dev=${TENSORRT_VERSION}" \
        "libnvinfer-bin=${TENSORRT_VERSION}" && \
    apt-mark hold libnvinfer10 libnvinfer-plugin10 libnvonnxparsers10 && \
    rm -rf /var/lib/apt/lists/*

ENV PATH=/usr/src/tensorrt/bin:${PATH}

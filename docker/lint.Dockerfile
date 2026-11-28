# Lightweight pre-commit runner for `./watod lint`.
# Runs the exact hooks defined in .pre-commit-config.yaml without installing
# anything on the host or bloating ROS module images. Hook environments are
# cached in a persistent Docker volume mounted at /cache/pre-commit.
FROM python:3.12-slim

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      ca-certificates \
      git \
      libxml2-utils && \
    rm -rf /var/lib/apt/lists/* && \
    pip3 install --no-cache-dir "pre-commit>=3.7,<5" && \
    git config --system --add safe.directory '*' && \
    mkdir -p /cache/pre-commit && \
    chmod 1777 /cache /cache/pre-commit

ENV PRE_COMMIT_HOME=/cache/pre-commit \
    PYTHONUNBUFFERED=1

WORKDIR /repo
ENTRYPOINT ["pre-commit"]

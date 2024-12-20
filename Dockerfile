# syntax=docker/dockerfile:1
FROM ubuntu:jammy AS upstream
# Restate for later use
ARG REPO

# prevent interactive messages in apt install
ARG DEBIAN_FRONTEND=noninteractive

# install development tools
RUN apt-get update \
    && apt-get install -q -y --no-install-recommends \
        apt-utils \
        build-essential \
        ccache \
        clang \
        clang-format \
        clang-tidy \
        cmake \
        gdb \
        git \
        libsfml-dev \
        libeigen3-dev \
        libgtest-dev \
        python3-pip \
        sudo \
        vim \
        wget \
        ssh-client \
    && rm -rf /var/lib/apt/lists/*

# copy source to install repo dependencies
WORKDIR /${REPO}

FROM upstream AS development

ARG UID
ARG GID
ARG USER

# fail build if args are missing
# hadolint ignore=SC2028
RUN if [ -z "$UID" ]; then echo '\nERROR: UID not set. Run \n\n \texport UID=$(id -u) \n\n on host before building Dockerfile.\n'; exit 1; fi
# hadolint ignore=SC2028
RUN if [ -z "$GID" ]; then echo '\nERROR: GID not set. Run \n\n \texport GID=$(id -g) \n\n on host before building Dockerfile.\n'; exit 1; fi
# hadolint ignore=SC2028
RUN if [ -z "$USER" ]; then echo '\nERROR: USER not set. Run \n\n \texport USER=$(whoami) \n\n on host before building Dockerfile.\n'; exit 1; fi

# install developer tools
RUN --mount=type=cache,target=/var/cache/apt,id=apt \
    apt-get update && apt-get upgrade -y \
    && apt-get install -q -y --no-install-recommends \
        clang-format \
        clang-tidy \
        git \
        openssh-client \
        vim \
        wget \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir \
    pre-commit==3.0.4

# Setup user home directory
# --no-log-init helps with excessively long UIDs
RUN groupadd --gid $GID $USER \
    && useradd --no-log-init --uid $GID --gid $UID -m $USER --groups sudo \
    && echo $USER ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USER \
    && chmod 0440 /etc/sudoers.d/$USER \
    && touch /home/${USER}/.bashrc \
    && chown -R ${GID}:${UID} /home/${USER}

USER $USER
ENV SHELL /bin/bash
ENTRYPOINT []

# Setup mixin
WORKDIR /home/${USER}/ws

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
        git \
        libsdl2-dev \
        libeigen3-dev \
        python3-pip \
        vim \
        wget \
        ssh-client \
    && rm -rf /var/lib/apt/lists/*

# install some pip packages needed for testing
RUN python3 -m pip install -U \
    pre-commit

# copy source to install repo dependencies
WORKDIR /${REPO}

FROM upstream AS development

ARG USER_UID
ARG USER_GID
ARG USER

# fail build if args are missing
RUN if [ -z "$USER_UID" ]; then echo '\nERROR: USER_UID not set. Run \n\n \texport USER_UID=$(id -u) \n\n on host before building Dockerfile.\n'; exit 1; fi
RUN if [ -z "$USER_GID" ]; then echo '\nERROR: USER_GID not set. Run \n\n \texport USER_GID=$(id -g) \n\n on host before building Dockerfile.\n'; exit 1; fi
RUN if [ -z "$USER" ]; then echo '\nERROR: USER not set. Run \n\n \texport USER=$(whoami) \n\n on host before building Dockerfile.\n'; exit 1; fi

# chown working directory to user
RUN mkdir -p /home/${USER}/${REPO} && chown -R ${USER_UID}:${USER_GID} /home/${USER}

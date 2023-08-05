FROM ubuntu:22.04 as python_base
ARG VERSION
ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

# Set the locale
ENV LC_ALL en_US.UTF-8
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US.UTF-8

ARG USERNAME=user
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo tzdata \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/* 

RUN apt-get update && apt-get install -y \
    python3 \
    && apt-get -y clean && rm -rf /var/lib/apt/lists/*

# WORKDIR /home/user
# COPY ./requirements.txt /home/user/requirements.txt
# RUN pip install --no-`cache-dir --upgrade -r /home/user/requirements.txt

# ################################################

FROM python_base as development

RUN apt-get update && apt-get install -y \
    git \
    python3-pip \
    python3-all \
    debhelper \
    dh-python \
    python3-stdeb \
    locales \
    locales-all \
    && apt-get -y clean && rm -rf /var/lib/apt/lists/*

RUN echo ${VERSION} > /tmp/version

# WORKDIR /home/user
# COPY ./requirements.txt /home/user/requirements.txt
# COPY ./requirements-dev.txt /home/user/requirements-dev.txt
# RUN pip install --no-`cache-dir --upgrade -r /home/user/requirements-dev.txt
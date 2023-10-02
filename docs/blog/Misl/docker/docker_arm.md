---
tags:
    - docker
    - arm
---
Build ARM docker image on x86 machine

```bash
sudo apt install qemu-user-static binfmt-support
update-binfmts --enable qemu-arm
update-binfmts --display qemu-arm
```


## docker
pull ubuntu for ARM

```bash
docker pull arm64v8/ubuntu
```

```bash
docker run -it --rm \
--name arm \
--hostname arm \
arm64v8/ubuntu \
/bin/bash

```

---

## Demo

Build ARM64 docker image base on ubuntu with python and none root user

```Dockerfile
FROM arm64v8/ubuntu:latest as python_base
ARG version
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
    python3-pip \
    python3-venv \
    vim \
    iputils-ping \
    net-tools \
    && apt-get -y clean && rm -rf /var/lib/apt/lists/*

ENV PATH=${PATH}:/home/user/.local/bin
```
#### Build
```bash
docker build -t ubuntu/arm:python -f Dockerfile .

```

#### Run 
```bash
docker run -it --rm \
--name arm \
--hostname arm \
--user user \
ubuntu/arm:python \
/bin/bash
```


!!! tip ""
    Disabled BuilderX

    `DOCKER_BUILDKIT=0` 
     
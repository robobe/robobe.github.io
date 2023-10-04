---
tags:
    - docker
    - arm
---
## Install docker on ARM

```bash
sudo apt-get -y install apt-transport-https ca-certificates curl software-properties-common

curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -

sudo add-apt-repository "deb [arch=arm64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"

sudo apt update

sudo apt-get -y install docker-ce

```

### Add user to docker group

```bash
sudo usermod -aG docker ${USER}
```

---

## Build image on X86 machine
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
FROM ubuntu:22.04
ARG version
ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

ARG USERNAME=user
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo tzdata locales \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/* 

# Set the locale
RUN sed -i -e 's/# en_US.UTF-8 UTF-8/en_US.UTF-8 UTF-8/' /etc/locale.gen && locale-gen
ENV LC_ALL en_US.UTF-8
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US.UTF-8

# Set user password
RUN echo 'user:user' | chpasswd

RUN apt-get update && apt-get install -y \
    python3 \
    python3-pip \
    vim \
    iputils-ping \
    net-tools \
    openssh-server \
    && apt-get -y clean && rm -rf /var/lib/apt/lists/*


ENV PATH=${PATH}:/home/user/.local/bin

COPY .devcontainer/id_ed25519.pub /home/user/.ssh/authorized_keys
RUN chown user:user /home/user/.ssh/authorized_keys && chmod 600 /home/user/.ssh/authorized_keys

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
     
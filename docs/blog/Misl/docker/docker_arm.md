---
tags:
    - docker
    - arm
    - qemu
---
# Run docker ARM container on x86 machine

Run ARM docker container on x86 using qemu


---

## Qemu
QEMU is a machine emulator, **qemu-user-static** utilizes QEMU's ability to perform dynamic binary translation. This means it can interpret the instructions of one CPU architecture and translate them into instructions that another CPU architecture can understand.
**binfmt** tells the Linux kernel how to execute binaries of foreign architectures using QEMU's static interpreters.

```bash
sudo apt install qemu-user-static binfmt-support
update-binfmts --enable qemu-arm
update-binfmts --display qemu-arm
```

---

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

```bash title=""
uname -a
#
Linux arm 6.5.0-41-generic #41~22.04.2-Ubuntu SMP PREEMPT_DYNAMIC Mon Jun  3 11:32:55 UTC 2 aarch64 aarch64 aarch64 GNU/Linux

```

---

## Demo
Build ARM docker image on x64 machine using buildx

```
sudo apt install docker-buildx
```

```Dockerfile
FROM arm64v8/ubuntu

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

```

```bash
docker buildx build \
  --platform "linux/arm64" \
  --tag arm:test \
  .
```

```bash
docker run -it --rm \
--name arm \
--hostname arm \
--user user \
arm:test \
/bin/bash
```
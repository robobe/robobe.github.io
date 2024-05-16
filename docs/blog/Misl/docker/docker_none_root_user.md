---
title: Docker with none ROOT user
tags:
    - docker
    - template
    - 
---

Create docker image base on ubuntu 22.04  
Add user `user` to image



### Dockerfile

- Add user `user` to image
- Install `sudo`
- Add user to `sudoers`
- Set prompt


```dockerfile
FROM ubuntu:22.04 AS base

ENV DEBIAN_FRONTEND=noninteractive

ARG USERNAME=user
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create a non-root user
RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  # [Optional] Add sudo support for the non-root user
  && apt-get update \
  && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  # Cleanup
  && rm -rf /var/lib/apt/lists/* 
```

---

### Usage
Run image with `user` context set `hostname` and `workdir`
#### cli
```bash
docker run -it \
--rm \
--user user \
--workdir /home/user \
--hostname dev \
ubuntu:user \
```



#### docker-compose

```yml
version: "3.0"
services:
  ubuntu:
    image: ubuntu:user
    hostname: dev
    user: user
    working_dir: /home/user
```

---


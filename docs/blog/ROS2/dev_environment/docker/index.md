---
title: Docker image
tags:
    - ros2
    - docker
---

!!! note ""
     Docker file base on [Allison Thackston ](https://github.com/athackst/dockerfiles) work

## Objective
- Create ROS2 humble docker image
- Run image and share simple workspace with simple pkg that contain minimal pub sub
    - Run from cli
    - Run with docker-compose

## Usage
- Run docker from cli

!!! note "shm"
    Share `/dev/shm` between host and container
     
### cli
```python title="run command" linenums="1" hl_lines="9"
docker run --rm -it \
-e DISPLAY=$DISPLAY \ \
--user user \
--workdir /home/user \
--hostname dev \
--net host \
--volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
--volume /home/user/ros2_ws:/home/user/ros2_ws \
--volume /dev/shm:/dev/shm \
humble:dev \
bash
```

### docker-compose
``` linenums="1" hl_lines="15"
version: "3.0"
services:
  pub:
    image: humble:dev
    hostname: dev
    user: user
    working_dir: /home/user
    stdin_open: true
    tty: true 
    environment:
      - DISPLAY=${DISPLAY}
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix
      - /home/user/ros2_ws:/home/user/ros2_ws
      - /dev/shm:/dev/shm
    network_mode: host
```
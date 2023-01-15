---
title: Docker image
tags:
    - ros2
    - docker
---

!!! note ""
     Docker file base on [Allison Thackston ](https://github.com/athackst/dockerfiles) work

# LAB
- Create ROS2 humble docker image
- Run image and share simple workspace with simple pkg that contain minimal pub sub
    - Run from cli
    - Run with docker-compose

# CLI
- Run docker from cli

!!! note "shm"
    Share `/dev/shm` between host and container
     
```python title="run command" linenums="1" hl_lines="7"
docker run --rm -it \
--env DISPLAY \
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

```
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
    command: cd ros2_ws; source install/setup.bash; ros2 run pkg_python_tutorial pub
```
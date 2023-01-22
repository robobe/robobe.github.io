---
title: Run Dockerize ROS2 Humble on RPI4
tags:
    - ros2
    - docker
    - rpi
---

- Install and Run docker on RPI
- Config devcontainer on local dev
- Attach / Run to docker container


![](images/dev_rpi_docker.drawio.png)

## ros2 docker
```
docker run -it --rm ros:humble-ros-base /bin/bash
```

## vscode devcontainer

```json title="settings.json"
{
    "docker.host": "ubuntu@<rpi ip>"
}
```

```json title="devcontainer.json"
{
	"name": "VS Code Remote docker PI",

	"context": "..",
	"dockerFile": "../docker/Dockerfile",

	"settings": { 
		"terminal.integrated.shell.linux": null
	},

	"extensions": [],

	"workspaceFolder": "/pi_ws",
	"workspaceMount": "source=/home/user/pi_ws/,target=/pi_ws,type=bind,consistency=cached",
}

```

## docker port 2375

!!! tip "port 2375"
    ```
    exec socat -d TCP-LISTEN:2375,fork UNIX-CONNECT:/var/run/docker.sock
    ```

     
!!! tip "start ssh without systemd"
    ```
    /etc/init.d/ssh start
    ```
     
---

# Reference
- [Here's How to Install Docker on Raspberry Pi?](https://www.simplilearn.com/tutorials/docker-tutorial/raspberry-pi-docker#installing_docker_raspberry_pi_4)
 -[Enable TCP port 2375 for external connection to Docker](https://gist.github.com/styblope/dc55e0ad2a9848f2cc3307d4819d819f)
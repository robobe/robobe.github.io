---
title: Install docker with nvidia support
description: Create boot able disk  from image
date: "2022-06-03"
banner: ../images/logo.jpeg
tags:
    - docker
    - nvidia
---

!!! Prerequisite
    Install nvidia driver on host machine
    
- Install docker follow `digitalocean`
- Install `nvidia-container-toolkit`
- Check

## Install
### nvidia-container-toolkit

```bash
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
# key
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
# apt
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
# install
sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
```

!!! Note
    nvidia-docker.list point to `ubuntu 18.04`

## Checking

```bash title="first run"
docker run --gpus all nvidia/cuda:10.0-base nvidia-smi
Unable to find image 'nvidia/cuda:10.0-base' locally
10.0-base: Pulling from nvidia/cuda
25fa05cd42bd: Pull complete 
2d6e353a95ec: Pull complete 
df0051b6f25d: Pull complete 
ad1e3e71b0c0: Pull complete 
d26e14de793e: Pull complete 
Digest: sha256:2f608ac9f6c6a9abb34112ccd85058877f669d4c976d7e3fc1c7cafcf5ef7dff
Status: Downloaded newer image for nvidia/cuda:10.0-base
Fri Jun  3 05:56:41 2022       
+-----------------------------------------------------------------------------+
| NVIDIA-SMI 510.73.05    Driver Version: 510.73.05    CUDA Version: 11.6     |
|-------------------------------+----------------------+----------------------+
| GPU  Name        Persistence-M| Bus-Id        Disp.A | Volatile Uncorr. ECC |
| Fan  Temp  Perf  Pwr:Usage/Cap|         Memory-Usage | GPU-Util  Compute M. |
|                               |                      |               MIG M. |
|===============================+======================+======================|
|   0  NVIDIA GeForce ...  Off  | 00000000:58:00.0 Off |                  N/A |
| N/A   46C    P0    N/A /  N/A |      9MiB /  2048MiB |      0%      Default |
|                               |                      |                  N/A |
+-------------------------------+----------------------+----------------------+
                                                                               
+-----------------------------------------------------------------------------+
| Processes:                                                                  |
|  GPU   GI   CI        PID   Type   Process name                  GPU Memory |
|        ID   ID                                                   Usage      |
|=============================================================================|
+-----------------------------------------------------------------------------+

```

!!! Note
    `--gps all ` arg in docker run command

### gui

```bash
xhost +local:docker
docker run --gpus all -it \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    nathzi1505:darknet bash
```

---

# Reference
- [How To Install and Use Docker on Ubuntu 20.04](https://www.digitalocean.com/community/tutorials/how-to-install-and-use-docker-on-ubuntu-20-04)
- [nvidia docker](https://nvidia.github.io/nvidia-docker/)
- [docker nvidia](https://gist.github.com/nathzi1505/d2aab27ff93a3a9d82dada1336c45041)
- [How to get your CUDA application running in a Docker container](https://www.celantur.com/blog/run-cuda-in-docker-on-linux/)
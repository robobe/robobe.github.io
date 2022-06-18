---
title: TensorRT first step
description: TensorRT first step
date: "2022-06-11"
banner: ../images/nvidia.png
tags:
    - nvidia
    - tensorrt
---

TensorRT is a machine learning framework that is published by Nvidia to run inference. TensorRT is highly optimized to run on NVIDIA GPUs


## Install
Install on x86 ubuntu 20.04 machine

### Prerequisite
- nvidia driver (check with `nvidia-smi` if working)
- cuda 

### Install TensorRT

- Download version from nvidia dev site (authentication needed)

!!! note "EA vs GA"
    - EA: Early access
    - GA: Stable version
     
![](images/tensorrt_download.png)


```bash
# sudo dpkg -i nv-tensorrt-repo-<ubuntu version>-<cuda version>-<...>
sudo dpkg -i nv-tensorrt-repo-ubuntu2004-cuda11.4-trt8.2.5.1-ga-20220505_1-1_amd64.deb
sudo apt-key add /var/nv-tensorrt-repo-ubuntu2004-cuda11.4-trt8.2.5.1-ga-20220505/82307095.pub
sudo apt update
sudo apt install tensorrt
sudo apt-get install python3-libnvinfer-dev
sudo apt-get install onnx-graphsurgeon
```

---

# References

- [What is TensorRT?](https://blog.roboflow.com/what-is-tensorrt/)
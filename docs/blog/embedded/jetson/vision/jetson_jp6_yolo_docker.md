---
tags:
    - jetson
    - nvidia
    - yolo
    - docker
    - pytorch
---

# Use YOLO on docker running on jetson orin with Jetpack 6.0

[Quick Start Guide: NVIDIA Jetson with Ultralytics YOLOv8](https://docs.ultralytics.com/guides/nvidia-jetson/#flash-jetpack-to-nvidia-jetson)


[github](https://github.com/ultralytics/ultralytics/blob/main/docker/Dockerfile-jetson-jetpack6)

```bash
t=ultralytics/ultralytics:latest-jetson-jetpack6
docker pull $t 
docker run -it --ipc=host --runtime=nvidia $t
```

```bash title="torch with cuda"
>>> import torch
>>> torch.cuda.is_available()
True
```

```bash title="tensorrt"
>>> import tensorrt
>>> tensorrt.__version__
'8.6.2'

```
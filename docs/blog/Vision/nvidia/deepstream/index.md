---
tags:
    - nvidia
    - deepstream
    - jetson
---




# Deepstream

NVIDIA’s DeepStream SDK is a complete streaming analytics toolkit based on GStreamer for AI-based multi-sensor processing [more](https://developer.nvidia.com/deepstream-sdk)


## install
### pc / docker

```bash
docker pull nvcr.io/nvidia/deepstream:7.0-samples-multiarch
```

```bash
docker run --gpus all -it --rm --net=host --privileged \
-v /tmp/.X11-unix:/tmp/.X11-unix \
-e DISPLAY=$DISPLAY \
-w /opt/nvidia/deepstream/deepstream-7.0 \
nvcr.io/nvidia/deepstream:7.0-samples-multiarch
```

```bash title=check
deepstream-app --version
#
deepstream-app version 7.0.0
DeepStreamSDK 7.0.0
```

### jetson
- jetpack 6.0 
- jetson orin nx

```bash
sudo apt install libgstrtspserver-1.0-0 
#
sudo apt install deepstream-7.0
```


---

## Reference
- [ How to use DeepStream with Jetson Orin Nano and ROS2 ](https://www.youtube.com/watch?v=vDxL2-YJcSY)
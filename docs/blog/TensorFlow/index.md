---
tags:
    - tensorflow
    - tf
---


## Docker

```
docker pull tensorflow/tensorflow:latest-gpu
```

```bash
docker run -it --rm \
-u $(id -u):$(id -g) \
--hostname tf \
-p 8888:8888 tensorflow/tensorflow:latest-gpu
```
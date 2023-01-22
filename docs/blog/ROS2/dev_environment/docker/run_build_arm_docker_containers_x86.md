---
title: Running and Building ARM Docker Containers on x86 
tags:
    - ros2
    - docker
    - arm
    - qemu
---

```bash
sudo apt-get install \
    qemu \
    binfmt-support\
    qemu-user-static 
```

```
docker run --rm \
--platform linux/aarch64 \
-t arm64v8/ubuntu uname -m 
```
---

## Reference
- [Running and Building ARM Docker Containers on x86 ](https://www.stereolabs.com/docs/docker/building-arm-container-on-x86/)



---
title: Docker Volumes
tags:
    - docker
---

Data volume help as to separate the container and data  
There are 2 way to manage data in docker
- Data volumes
- Data volume container

## Data volumes
### Bind mount / Mounting host directory as a data volume

bind mount is to mount an existing directory or file on the host to the container.  

```
-v HostFolder:ContainerVolumeName
```

#### demo
mount share folder in user home directory on container root (/) named data

```bash
docker run -it --name b1 -v /home/user/share:/data busybox
# create file with touch and exit
# check file exists on host folder
```

---

### Docker managed volume

```bash
# no need to specify the mount source
-v ContainerVolumeName
```
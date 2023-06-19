---
tags:   
    - aptly
    - ros2
---

# Aptly Demo: mirror ROS2 humble repository

## create mirror
### install key
```bash
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

### import 

```bash
gpg --no-default-keyring --keyring /usr/share/keyrings/ros-archive-keyring.gpg --export | gpg --no-default-keyring --keyring trustedkeys.gpg --import
```

### create 
```bash title="create mirror"
aptly mirror -architectures="amd64" \
create ros2-humble  \
http://packages.ros.org/ros2/ubuntu/ jammy main 
```

## update
```
aptly mirror update ros2-humble
```
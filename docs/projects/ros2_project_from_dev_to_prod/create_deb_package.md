---
title: Create debian package from ROS package
tags:
    - ros2
    - projects
    - deploy
---

## Prerequisites
- Set `package.xml` with all dependencies
- Init rosdep and install all package dependencies

!!! note "rosdep"
    rosdep is a command-line tool for installing system dependencies. 
     

### install
```bash
sudo apt install \
    python3-bloom \
    python3-rosdep \
    fakeroot \
    debhelper \
    dh-python
```

### init rosdep

```
sudo rosdep init
resdep update
```

## usage

```bash
cd <package root folder>
# run 
bloom-generate rosdebian
# run
fakeroot debian/rules binary
```

Assuming that all required dependencies are available and that compilation succeeds, the new package will be available in the parent directory of this directory.


# Reference
- [Building a custom Debian package](https://docs.ros.org/en/humble/How-To-Guides/Building-a-Custom-Debian-Package.html)
---
tags:
    - ros2
    - deb
    - package
---
# Building ROS2 custom debian package
Build debian deb file from ROS2 package

## install
```bash
sudo apt install python3-bloom python3-rosdep fakeroot debhelper dh-python
```

## prerequisites
- Package dependencies must be install
- All dependencies must be declared in `package.xml` package file


## bloom
Bloom is a build automation tool

## Changelog
A changelog is a file that contains a condensed list of all important changes made to a project 

The changelog is a list of commits

```bash
git log --oneline
```

!!! tip "catkin_generate_changelog"
    ```bash
    catkin_generate_changelog --all
    ```
     
## package version
ROS2 python package has two files with version fields
- package.xml
- setup.py

To maintain version use `catkin_prepare_release` that maintain both files and create tag and change log

!!! note ""
     Run from workspace root

```bash
catkin_prepare_release --version 0.0.2 -y --no-push
```


!!! tip "version semantic"
    Version number has three parts
    ```
    Major.Minor.Path
    ```
     
## create package
from package root folder (not workspace)

```bash
bloom-generate rosdebian
```

```bash
fakeroot debian/rules binary
```

!!! note "deb file"
     create package in the parent folder
---


# Reference
- [Building a custom Debian package](https://docs.ros.org/en/humble/How-To-Guides/Building-a-Custom-Debian-Package.html)
- [How to release a ROS 2 binary package – Part 3](https://www.theconstructsim.com/how-to-release-a-ros-2-binary-package-part-3/)
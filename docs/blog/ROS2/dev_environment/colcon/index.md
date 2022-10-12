---
title: Colcon build system
description: Command line tool to build sets of software packages
date: "2022-03-29"
banner: colcon.png
tags:
    - ros2
    - colcon
---
## Colcon
colcon is a command line tool to improve the workflow of building, testing and using multiple software packages. It automates the process, handles the ordering and sets up the environment to use the packages.

## Build tools
Build tools are programs that automate the creation of executable applications from source code
- Compiling
- Linking
- Packaging

## Build system
- Repeatable: ensure that that the project is built exactly the same way every time it builds
- Reproducible: easily recreate (reproduce) the steps that are required to perform a build
- Standard ensure that all of your projects follow the same steps and implement best practices


!!! Note
    - Make
    - Ninja
  
## Generator
- CMake: create build system for specific O.S or system

## Meta build


## Install
```
sudo apt install python3-colcon-common-extensions
```

## Usage
**In the root of the workspace** run `colcon build`


## References
- [ROS2 foxy Using colcon to build packages](https://docs.ros.org/en/foxy/Tutorials/Colcon-Tutorial.html#install-colcon)
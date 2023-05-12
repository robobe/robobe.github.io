---
tags:
    - ros2
    - setup
    - local_setup
---

# ROS2 workspace
**Workspace** is a ROS term for the location on your system where you’re developing with ROS 2
The core ROS 2 workspace is called the **underlay**. Subsequent local workspaces are called **overlays**.

**underlay workspace** contain all binary package that install by `apt` or `rosdep` (that run apt)

**overlays workspace** contain source packages get download/clone or develop
when we use `colcon` to build the workspace, it run on the source from `src` folder build into `build` folder and install the result into `install` folder.
colcon create `setup` and `local_setup` files with all the information need to find workspace packages in ROS environment

## Overlaying ROS Workspaces



Difference between local_setup.<ext> and setup.<ext>
[ROS answers](https://answers.ros.org/question/292566/what-is-the-difference-between-local_setupbash-and-setupbash/
)
- The `local_setup` script sets up environment for all packages in the script prefix path
The `setup` source the local and other workspace


## Reference
- [ROS Overview (10 concepts you need to know)](https://articulatedrobotics.xyz/ready-for-ros-4-ros-overview/)
- [ROS 2 workshop](https://ros2-industrial-workshop.readthedocs.io/en/latest/index.html)


---
title: tf2
tags:
    - ros2
---

## transformations
Coordinate transformations are a a mathematical tool to convert/represent one set of coordinate to other.


## tf
Any node can use the tf2 libraries to broadcast a transform from one frame to another

each frame is defined by **one** (and only one) transform from another frame, but can have any number of frames dependent on it, this create a tree structure of `frames` the library can calculat any transformation between tow frames (nodes)

`tf2` library using topics `/tf` and `/tf_static` th handle communication, because the tf2 library pub/sub the messages we call it `broadcasting` and `listening`

---

## install tf2 tools and helper libraries
```
sudo apt-get install \
ros-humble-tf2-tools \
ros-humble-tf-transformations \
ros-humble-rqt-tf-tree
```

## tutorials
- [cpp broadcaster](tf2_cpp_broadcaster.md)

# Reference
- [Getting Ready for ROS Part 6: The Transform System (TF)](https://articulatedrobotics.xyz/ready-for-ros-6-tf/)
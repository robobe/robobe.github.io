---
title: Part1 - launch gazebo
description: launch gazebo simple example
date: "2022-05-02"
tags:
    - gazebo
    - launch
    - 101
---

# gazebo_ros
Provides ROS plugins that offer message and service publishers for interfacing with Gazebo through ROS.

```bash
sudo apt install ros-foxy-gazebo-ros
```
---
## project
[my ros2_gazebo_tutorial](https://github.com/robobe/ros2_gazebo_tutorial)

```
├── CMakeLists.txt
├── launch
│   └── basic_gazebo.launch.py
├── package.xml
└── worlds
    └── empty.world

```

---

## world

```xml title="empty.world"
<?xml version='1.0'?>
<sdf version="1.6">
<world name="room">
  <include>
    <uri>model://sun</uri>
  </include>
  <include>
    <uri>model://ground_plane</uri>
  </include>
</world>
</sdf>
```

## launch

```python title="basic_gazebo.launch.py" linenums="1" hl_lines="25 31 35"
--8<-- "/home/user/ros2_ws/src/ros2_gazebo_tutorial/launch/basic_gazebo.launch.py"
```

## cmake
- Copy launch and world folders 


```cmake title="CMakeLists.txt"
# Add this line to CMakeLists.txt

install(DIRECTORY
  launch
  worlds
  DESTINATION share/${PROJECT_NAME}/
)
```

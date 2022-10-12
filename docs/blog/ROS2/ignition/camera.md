---
title: Ignition ROS2 camera bridge
tags:
    - ignition
    - ros2
    - camera
    - bridge
---

- IGNITION: fortress
- ROS2: humble


## Objective
- Create simulation with camera
- Publish camera using ros_ign bridge
- Write launch file
- View camera image in RVIZ

## Bridge
Bridge a collection of ROS2 and ignition Transport topics and services.

- @  == a bidirectional bridge.
- [  == a bridge from Gazebo to ROS.
- ]  == a bridge from ROS to Gazebo.

```bash
ros2 run ros_gz_bridge parameter_bridge --help

# bidirectional bridge example:
ros2 run ros_gz_bridge parameter_bridge /chatter@std_msgs/String@ignition.msgs.StringMsg

# bridge from Gazebo to ROS example:
ros2 run ros_gz_bridge parameter_bridge /chatter@std_msgs/String[ignition.msgs.StringMsg

# bridge from ROS to Gazebo example:
ros2 run ros_gz_bridge parameter_bridge /chatter@std_msgs/String]ignition.msgs.StringMsg

# bridge with remapping
 --ros-args -r /<ign topic>:=/<ros2 topic>
 --ros-args -r /lidar2:=/laser_scan
```




---


## Rviz
- [Rviz types](http://wiki.ros.org/rviz/DisplayTypes)

| type  | Desc  |
|---|---|
| Camera  | use `CameraInfo` to create window in show the image  |
| Image  | display image without `CameraInfo` data  |


---

## launch

```python

```
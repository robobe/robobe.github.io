---
title: Moving an object in Gazebo using ROS2 service
tags:
    - gazebo
    - ros2
---

# LAB
- Move Gazebo entities from using ROS2 service


### world
```xml
<plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
    <ros>
    <namespace>/gazebo</namespace>
    </ros>

    <update_rate>1.0</update_rate>
</plugin>
```

### get

```bash
ros2 service call /demo/get_entity_state gazebo_msgs/srv/GetEntityState "{name: camera_top,reference_frame: world}"
```

### set

```bash
ros2 service call /demo/set_entity_state gazebo_msgs/srv/SetEntityState "state: {name: camera_top, pose: {position:{x: 2.0, y: 2.0, z: 5.0}}, reference_frame: world}"
```

```
ros2 service call /demo/set_entity_state gazebo_msgs/srv/SetEntityState \
"state: {name: camera_top, pose: \
{position:{x: 2.0, y: 2.0, z: 5.0},
orientation:{x: 0.0, y: 0.706825181105366, z: 0.0, w: 0.7073882691671998}}, \
reference_frame: world}"
```


```
ros2 service call /set_entity_state gazebo_msgs/srv/SetEntityState "state: {name: landmark, \
pose: {position:{x: 1.0, y: 0.0, z: 2.0}}, \
reference_frame: world}"
```
---

# Reference
- [ROS 2 Migration: Entity states](https://github.com/ros-simulation/gazebo_ros_pkgs/wiki/ROS-2-Migration:-Entity-states)


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
<sdf version="1.6">
<world name="default">
    <include>
        <uri>model://ground_plane</uri>
    </include>
    <include>
        <uri>model://sun</uri>
    </include>
    <!-- models-->
    <model name="cube">
        <static>true</static>
        <link name="link">
            <pose>0 0 2.5 0 0 0</pose>
            <visual name="visual">
                <geometry>
                    <box>
                        <size>2 1 1</size>
                    </box>
                </geometry>
            </visual>
        </link>
    </model>
    <!-- plugins -->
    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
        <ros>
            <namespace>/demo</namespace>
            <argument>model_states:=model_states_demo</argument>
        </ros>
        <update_rate>1.0</update_rate>
    </plugin>
</world>
</sdf>
```

### get

```bash
ros2 service call /demo/get_entity_state gazebo_msgs/srv/GetEntityState "{name: cube::link,reference_frame: world}"
```

### set

```bash
ros2 service call /demo/set_entity_state gazebo_msgs/srv/SetEntityState "state: {name: cube::link, pose: {position:{x: 2.0, y: 2.0, z: 5.0}}, reference_frame: world}"
```

```
ros2 service call /demo/set_entity_state gazebo_msgs/srv/SetEntityState \
"state: {name: cube::link, pose: \
{position:{x: 0.0, y: 0.0, z: 2.5},
orientation:{x: 0.7071, y: 0.0, z: 0.7071, w: 0.0}}, \
reference_frame: world}"
```

---

# Reference
- [ROS 2 Migration: Entity states](https://github.com/ros-simulation/gazebo_ros_pkgs/wiki/ROS-2-Migration:-Entity-states)


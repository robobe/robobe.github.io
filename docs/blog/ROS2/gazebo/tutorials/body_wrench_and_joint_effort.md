---
title: Body wrench and Joint Effort
tags:
    - gazebo
    - tutorial
---


# Link Wrench

model plugin: libgazebo_ros_force.so
system plugin: libgazebo_ros_force_system.so


## LAB : 

### libgazebo_ros_force

|       |                             |
| ----- | --------------------------- |
| world | gazebo_ros_force_demo.world |


```xml title="model force" linenums="1" hl_lines="1 7"
<plugin name="gazebo_ros_force" filename="libgazebo_ros_force.so">
    <ros>
        <namespace>/demo/world</namespace>
        <remapping>gazebo_ros_force:=force_demo</remapping>
    </ros>
    <link_name>link</link_name>
    <force_frame>world</force_frame>
</plugin>
```

```bash
# Apply force relative to force_frame
ros2 topic pub -1 /demo/world/force_demo geometry_msgs/Wrench "force: {x: 10.0}"

```

### libgazebo_ros_force_system.so

- Run gazebo with system pligin


```
gazebo --verbose -s libgazebo_ros_force_system.so force_demo.world
```

```bash
ros2 service list
#
/apply_joint_effort
/apply_link_wrench
/clear_joint_efforts
/clear_link_wrenches

```

```
ros2 service call /apply_link_wrench gazebo_msgs/srv/ApplyLinkWrench '{link_name: "force_on_world_frame::link", reference_frame: "", reference_point: { x: 0, y: 0, z: 0 }, wrench: { force: { x: 10, y: 0, z: 0 }, torque: { x: 0, y: 0, z: 0 } }, start_time: {sec: 0, nanosec: 0}, duration: {sec: -1, nanosec: 0} }'

ros2 service call /apply_link_wrench gazebo_msgs/srv/ApplyLinkWrench '{link_name: "force_on_world_frame::link", reference_frame: "force_on_world_frame::link", reference_point: { x: 0, y: 0, z: 0 }, wrench: { force: { x: 10, y: 0, z: 0 }, torque: { x: 0, y: 0, z: 0 } }, start_time: {sec: 0, nanosec: 0}, duration: {sec: -1, nanosec: 0} }'
```

```
ros2 service call /clear_link_wrenches gazebo_msgs/srv/LinkRequest '{link_name: "force_on_world_frame::link"}' 
```

TODO: 
- usage of this service
- reference name ?
- reference point
- force units
- why the box move only when we apply 10 and above
- what is the different between the system and the model plugin
---

# Reference

- [gazebo plugin worlds](/opt/ros/humble/share/gazebo_plugins/worlds)
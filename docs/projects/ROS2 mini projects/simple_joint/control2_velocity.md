---
title: Control2 joint velocity
tags:
    - control
    - gazebo_ros2_control
    - velocity_controller
---

Load and use velocity controller

#### launch: load controllers

```python
robot_controller_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=[
        "joint_velocity_controller", 
        "--controller-manager",
        "/controller_manager"
    ],
)
```

#### urdf control section
```xml
<ros2_control name="GazeboSystem" type="system">
    <hardware>
        <plugin>gazebo_ros2_control/GazeboSystem</plugin>
    </hardware>
    <joint name="base_to_second_joint">
        <command_interface name="velocity"/>
        <state_interface name="position"/>
        <state_interface name="velocity"/>
        <state_interface name="effort"/>
    </joint>
</ros2_control>

<gazebo>
    <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
        <parameters>$(find simple_joint)/config/velocity.yaml</parameters>
    </plugin>
</gazebo>
```

## usage
```
ros2 topic pub --once \
/joint_velocity_controller/commands  \
std_msgs/msg/Float64MultiArray \
"{data: [6.28]}"
```

---

## source

```yaml title="velocity.yaml"
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    joint_velocity_controller:
      type: velocity_controllers/JointGroupVelocityController

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

joint_velocity_controller:
  ros__parameters:
    joints:
      - base_to_second_joint
    interface_name: velocity
    command_interfaces:
      - velocity
    state_interfaces:
      - position
      - velocity
```
---
tags:
    - ros2
    - ros2_control
    - effort
---

# Effort controller
## urdf
```xml title="ros2 control"
<ros2_control name="GazeboSystem" type="system">
    <hardware>
        <plugin>gazebo_ros2_control/GazeboSystem</plugin>
    </hardware>
    <joint name="camera2base">
        <command_interface name="effort" />
        <state_interface name="position" />
        <state_interface name="velocity" />
        <state_interface name="effort" />
    </joint>
</ros2_control>
```

```xml title="load config"
<gazebo>
    <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
        <parameters>$(find gazebo_tutorial_pkg)/config/effort.yaml</parameters>
    </plugin>
</gazebo>
```


## config
```yaml title="" linenums="1" hl_lines="5 11"
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    joint_effort_controller:
      type: effort_controllers/JointGroupEffortController

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

joint_effort_controller:
  ros__parameters:
    joints:
      - camera2base
    interface_name: effort
    command_interfaces:
      - effort
    state_interfaces:
      - position
      - velocity

joint_state_broadcaster:
  ros__parameters:
      joints:
        - camera2base
```

## usage
### load controllers
```bash
ros2 run controller_manager spawner joint_effort_controller
```

!!! note "unload controller"
    using unspawn command to take command down
    ```
    ros2 run controller_manager unspawner joint_effort_controller
    ```
     

### topics
```bash linenums="1" hl_lines="2 3"
/clock
/dynamic_joint_states
/joint_effort_controller/commands
/joint_effort_controller/transition_event
/joint_state_broadcaster/transition_event

```

## usage

### command
```bash
ros2 topic pub --once \
/joint_effort_controller/commands \
std_msgs/msg/Float64MultiArray "{data: [10]}"
```
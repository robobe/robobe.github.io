---
title: Control
tags:
    - control
---
Controller nodes provide a convenient interface to control the joints of your robot.
Controller nodes are separated into three groups:
- Effort
- Velocity
- Position

---

## Demo (RRBot control)

```yaml title="controller_configuration.yaml"
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    forward_position_controller:
      type: forward_command_controller/ForwardCommandController

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

forward_position_controller:
  ros__parameters:
    joints:
      - joint1
      - joint2
    interface_name: position
```

### urdf
- Add ros2_control tag and gazebo plugin

```xml
 <ros2_control name="GazeboSystem" type="system">
    <hardware>
      <plugin>gazebo_ros2_control/GazeboSystem</plugin>
    </hardware>
    <joint name="joint1">
      <command_interface name="position"/>
      <state_interface name="position"/>
    </joint>
    <joint name="joint2">
      <command_interface name="position"/>
      <state_interface name="position"/>
    </joint>
  </ros2_control>

  <gazebo>
    <plugin name="gazebo_ros2_control" filename="libgazebo_ros2_control.so">
      <parameters>$(find rrbot_control)/config/controller_configuration.yaml</parameters>
    </plugin>
  </gazebo>
```
### usage

- Run gazebo with robot
- Start control from command line

```bash title="rrbot.launch.py"
ros2 launch rrbot_gazebo  rrbot.launch.py
```

```bash title="run contrllers"
ros2 control load_controller --set-state start joint_state_broadcaster
ros2 control load_controller --set-state start forward_position_controller
```

```bash title="position command"
ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data:
- 0.5
- 0.5"
```

![](images/after_forward_command.png)

---

## controller launch 

```python
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["forward_position_controller", "-c", "/controller_manager"],
    )

    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[robot_controller_spawner],
            )
        ),
        joint_state_broadcaster_spawner
    ])
```

---

# Reference
- [ros2_control_demo](http://www.lxshaw.com/tech/ros/2022/01/12/ros2_control_demo%E4%BB%A3%E7%A0%81%E8%A7%A3%E6%9E%90/)
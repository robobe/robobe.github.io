---
tags:
    - ros2
    - ros2_control
    - velocity
---

# Control joint using velocity command

### urdf
```xml title="load controller from urdf"
<ros2_control name="GazeboSystem" type="system">
    <hardware>
        <plugin>gazebo_ros2_control/GazeboSystem</plugin>
    </hardware>
    <joint name="camera2base">
        <command_interface name="velocity" />
        <state_interface name="position" />
        <state_interface name="velocity" />
        <state_interface name="effort" />
    </joint>
</ros2_control>

<gazebo>
    <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
        <parameters>$(find gazebo_tutorial_pkg)/config/velocity.yaml</parameters>
    </plugin>
</gazebo>
```

```xml title="load ros2_control plugin"
<gazebo>
    <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
        <parameters>$(find simple_joint)/config/velocity.yaml</parameters>
    </plugin>
</gazebo>
```

### config

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
      - camera2base
    interface_name: velocity
    command_interfaces:
      - velocity
    state_interfaces:
      - position
      - velocity

joint_state_broadcaster:
  ros__parameters:
      joints:
        - camera2base
```

### usage

```bash title="load controllers"
# joint_state
ros2 run controller_manager spawner joint_state_broadcaster

# velocity controller
ros2 run controller_manager spawner velocity_controller
```

OR 

```python title="load_controller.launch.py" linenums="1" hl_lines="8 16"
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()


    ros2_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    ros2_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_effort_controller'],
        output='screen'
    )

    ld.add_action(ros2_state_spawner)
    ld.add_action(ros2_controller_spawner)
    return ld

```

#### titles as cli

```bash title="topics" linenums="1" hl_lines="3 6"
...
/clock
/dynamic_joint_states
/joint_state_broadcaster/transition_event
/joint_states
/joint_velocity_controller/commands
/joint_velocity_controller/transition_event
/parameter_events

```

#### velocity command
```bash title="pub velocity command"
ros2 topic pub --once \
/joint_velocity_controller/commands \
std_msgs/msg/Float64MultiArray "{data: [0.707]}"
```

```bash
ros2 topic echo /dynamic_joint_states
#
---
header:
  stamp:
    sec: 305
    nanosec: 327000000
  frame_id: ''
joint_names:
- camera2base
interface_values:
- interface_names:
  - position
  - velocity
  - effort
  values:
  - 187.45397219177687
  - 0.707
  - 0.0
---

```


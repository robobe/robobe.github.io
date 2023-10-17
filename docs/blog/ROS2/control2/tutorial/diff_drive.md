---
tags:
    - ros2
    - control2
    - demo
    - diff-drive
    - ignition
---

# ROS2 Diff drive control demo

[github](https://github.com/ros-controls/gz_ros2_control/tree/humble/ign_ros2_control_demos)

```bash
sudo apt install ros-humble-ign-ros2-control-demos
```

```bash
ros2 launch ign_ros2_control_demos diff_drive_example.launch.py
```

## Demo: Move robot using teleop

```bash
ros2 topic list
#
/clock
/diff_drive_base_controller/cmd_vel_unstamped
/diff_drive_base_controller/odom

```

---

### Run teleop cli

```bash title="run teleop"
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
--ros-args --remap cmd_vel:=diff_drive_base_controller/cmd_vel_unstamped
```


### Run teleop from launch file

```python title="launch teleop"
import launch
from launch_ros.actions import Node


def generate_launch_description():
    ld = launch.LaunchDescription()
    teleop = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        output='screen',
        name="teleop_demo",
        prefix = 'xterm -e',
        remappings=[
            ('/cmd_vel', '/diff_drive_base_controller/cmd_vel_unstamped'),
        ]
    )


    ld.add_action(teleop)
    return ld
```

!!! tip "xterm -e"
    Running node that need `stdin` in separate terminal using

    ```python title="Node argument"
    prefix = 'xterm -e',
    ```
     

---

## tf tree

```bash
ros2 run rqt_tf_tree rqt_tf_tree
```

![](images/tf_tree_fid.png)

---

## Controllers
- joint_state_broadcaster
- diff_drive_base_controller

load controllers using `ExecuteProcess`

### Launch 
[check launch file](https://github.com/ros-controls/gz_ros2_control/blob/humble/ign_ros2_control_demos/launch/diff_drive_example.launch.py)

- Using event to trigger action

ignition_spawn_entity --> load_joint_state_controller -->
load_joint_trajectory_controller


```python
from launch.actions import ExecuteProcess

load_joint_trajectory_controller = ExecuteProcess(
    cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
            'diff_drive_base_controller'],
    output='screen'
)
```

```python
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

RegisterEventHandler(
    event_handler=OnProcessExit(
        target_action=load_joint_state_controller,
        on_exit=[load_joint_trajectory_controller],
    )
),
```


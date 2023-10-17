---
tags:
    - bridge
    - ros2
    - gazebo
    - gz
    - ros_gz_bridge
---

# Gazebo ROS2 bridge

This package provides a network bridge which enables the exchange of messages between ROS2 and Gazebo Transport.
[more](https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_bridge/README.md)

```bash
ros2 run ros_gz_bridge parameter_bridge <topic name>@<ros msg type>@<gazebo msg type>
```

---

## Simple demo
Send gazebo clock topic to ROS2 
from gazebo (garden) empty world

- Run bridge from CLI
- Run bridge from launch file
- Run bridge from launch with yaml config


```bash title="gz topics"
gz topic --list
#
/clock
/gazebo/resource_paths
/gui/camera/pose
/stats
...
```

### From CLI

```bash title="run bridge from cli"
ros2 run ros_gz_bridge parameter_bridge "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"
```

!!! tip "Closing the rule in quotation marks"
    In `zsh` without the quotation we get `zsh: bad pattern: ` error


#### message direction

| Direction |                        |
| --------- | ---------------------- |
| @         | Bridge both directions |
| [         | GZ->ROS Bridge         |
| ]         | ROS->GZ Bridge         |


```bash title="ros2 topics"
ros2 topic list
#
/clock
/parameter_events
/rosout
```

```bash title="echo topic"
ros2 topic echo /clock
#
clock:
  sec: 2832
  nanosec: 447000000
---
clock:
  sec: 2832
  nanosec: 448000000
```

---

### Using launch file

```python title="bridge.launch.py"
import launch
from launch_ros.actions import Node


def generate_launch_description():
    ld = launch.LaunchDescription()
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        output="screen",
    )

    ld.add_action(bridge)
    return ld
```

---

### Using launch with yaml file
Using yaml to config bridge mapping

| direction     |                                                  |
| ------------- | ------------------------------------------------ |
| BIDIRECTIONAL | Default "BIDIRECTIONAL" - Bridge both directions |
| GZ_TO_ROS     | Bridge Ignition topic to ROS                     |
| ROS_TO_GZ     | Bridge ROS topic to Ignition                     |

[more yaml config option](https://github.com/gazebosim/ros_gz/blob/humble/ros_gz_bridge/README.md#example-5-configuring-the-bridge-via-yaml)


```yaml title="bridge.yaml"
- ros_topic_name: "/clock"
  gz_topic_name: "/clock"
  ros_type_name: "rosgraph_msgs/msg/Clock"
  gz_type_name: "gz.msgs.Clock"
  direction: GZ_TO_ROS
```

```python title="bridge.launch.py"
import pathlib
import launch
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    ld = launch.LaunchDescription()
    pkg_project_bringup = get_package_share_directory("nav_bringup")
    bridge_config = pathlib.Path(pkg_project_bringup).joinpath("config", "gz_bridge.yaml").as_posix()
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {
                "config_file": bridge_config
            }
        ],
        output="screen",
    )

    ld.add_action(bridge)
    return ld

```

#### Using  ExecuteProcess
Run bridge with yaml config using `ExecuteProcee` action

```python
import pathlib
import launch
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess


def generate_launch_description():
    ld = launch.LaunchDescription()
    pkg_project_bringup = get_package_share_directory("nav_bringup")
    bridge_config = pathlib.Path(pkg_project_bringup).joinpath("config", "gz_bridge.yaml").as_posix()
    bridge = ExecuteProcess(
        cmd=[
            'ros2 run ros_gz_bridge parameter_bridge  --ros-args -p config_file:={}'.format(bridge_config)
            ],
        output='screen',
        shell=True
    )

    ld.add_action(bridge)
    return ld

```
---
tags:
    - ignition
    - ros2
    - bridge
---

## ROS2 ignition/gz bridge

ros_ign contains packages that provide integration between ROS2 and Ignition:

- ros_ign: Metapackage that provides all other software packages;
- ros_ign_image: Use image_transport to transfer the image from Ignition to the one-way transmission bridge of ROS;
- ros_ign_bridge: Two-way transmission bridge between Ignition and ROS;
- ros_ign_gazebo: It is convenient to use the startup files and executable files of Ignition Gazebo and ROS;
- ros_ign_gazebo_demos: Demos using ROS-Ignition integration;
- ros_ign_point_cloud: A plug-in used to simulate publishing point clouds to ROS from Ignition Gazebo

### install
```
sudo apt install ros-humble-ros-gz
```

---

### Demo
Pub keypress from gazebo **fortress** to ROS2 using bridge
- Load bridge with mapping
- Open Terminal with `ROS` echo command
- For testing open anther Terminal with `ign` echo command
- Run gazebo with `key press` plugin
- Press anywhere on gazebo , both subscriber window show the keypress code

```bash title="run bridge"
# <topic name>@<ros2 type>@<gz type>
ros2 run ros_gz_bridge parameter_bridge \
/keyboard/keypress@std_msgs/msg/Int32@ignition.msgs.Int32
```

```bash title="ros subscriber"
ros2 topic echo /keyboard/keypress
```

```bash title="ign echo (subscriber)"
ign topic -e --topic /keyboard/keypress
```

```bash
ign gazebo empty.sdf
```

![](images/key_press_plugin.png)
#### using yaml config

```yaml title="ign2ros_bridge.yaml"
- topic_name: /keyboard/keypress
  ros_type_name: std_msgs/msg/Int32
  ign_type_name: ignition.msgs.Int32
  direction: BIDIRECTIONAL
```

```bash name="Terminal1: Run bridge
ros2 run ros_gz_bridge \
bridge_node \
--ros-args \
-p config_file:=$PWD/src/gz_demos/config/ign2ros_bridge.yaml
```

```bash title="Terminal2 ROS echo"
ros2 topic echo /keyboard/keypress
```

```bash title="Terminal3 ign pub"
ign topic  -t /keyboard/keypress --msgtype ignition.msgs.Int32 -p "data: 102"

```

---

### Demo: using ROS2 launch file


```python title="simple_bridge.launch.py"
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # Bridge
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/keyboard/keypress@std_msgs/msg/Int32@ignition.msgs.Int32",
        ],
        output="screen",
    )

    ld.add_action(bridge)
    return ld

```

#### Pub from Gazebo to ROS2

```bash title="Terminal1"
ros2 launch gz_demos simple_bridge.launch.py
```

```bash title="Terminal2"
ros2 topic echo /keyboard/keypress
```

```bash title="Terminal3"
ign topic  -t /keyboard/keypress --msgtype ignition.msgs.Int32 -p "data: 1021"
```

#### Pub from ROS2 to Gazebo

```bash title="Terminal2"
ign topic -e --topic /keyboard/keypress
```

```bash title="Terminal3"
ros2 topic pub /keyboard/keypress std_msgs/msg/Int32 "{data: 100}"
```

---

### Demo: ROS2 launch and bridge YAML config

!!! note ""
     Example and idea from [gazebosim](https://answers.gazebosim.org/question/28675/how-to-use-ros_gz_bridge-in-roslaunch-with-conf-yaml-and-readme-issue/)

```python title="bridge with yaml"
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('gz_demos'),
        'config',
        'ign2ros_bridge.yaml'
        )
    
    # Bridge
    bridge = ExecuteProcess(
        cmd=[
            'ros2 run ros_gz_bridge parameter_bridge  --ros-args -p config_file:={}'.format(config)
            ],
        output='screen',
        shell=True
    )

    ld.add_action(bridge)
    return ld

```



# Reference
- [/ros_gz_bridge](https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_bridge)
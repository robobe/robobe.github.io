---
service
tags:
    - gazebo
    - ros2
    - plugins
---
# Moving an object in Gazebo using ROS2 service
Control gazebo entities (links) using ROS2 and `libgazebo_ros_state` gazebo plugin. this plugin has two services:
- get_entity_state
- set_entity_state

## LAB objective
- Get gazebo entity state using ROS2 service
- Set gazebo entities state using ROS2 service
- Set/Get from cli
- Control from python node

## project
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

### launch

!!! tip "gazebo environment variables"
    Don't forget to source  
    The launch file append path to `GAZEBO_RESOURCE_PATH` variable for `world` file location

    ```
    source /usr/share/gazebo/setup.sh
    ```
     

```python
import os
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

PACKAGE_NAME = "camera_calibration_gazebo"
WORLD = "demo.world"

def generate_launch_description():
    pkg_share = get_package_share_directory(PACKAGE_NAME)
    gazebo_pkg = get_package_share_directory("gazebo_ros")

    # source /usr/share/gazebo/setup.sh
    resources = [os.path.join(pkg_share, "worlds")]

    resource_env = AppendEnvironmentVariable(
        name="GAZEBO_RESOURCE_PATH", value=":".join(resources)
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(gazebo_pkg, "launch", "gazebo.launch.py")]
        ),
        launch_arguments={"verbose": "true", "world": WORLD}.items(),
    )

    ld = LaunchDescription()
    ld.add_action(resource_env)
    ld.add_action(gazebo)

    return ld
```

---
## cli
### get_entity_state

```bash
ros2 service call /demo/get_entity_state gazebo_msgs/srv/GetEntityState "{name: cube::link,reference_frame: world}"
```

### set_entity_state

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


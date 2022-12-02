---
title: ROS2 Gazebo
tags:
    - ign
    - gz
    - ros2
---

## gz environment variables
|   |   |
|---|---|
| IGN_GAZEBO_RESOURCE_PATH  |   |
| IGN_GAZEBO_SYSTEM_PLUGIN_PATH |  |

---

# ROS2

## ROS2 copy folder to install folder

```c
install(DIRECTORY
    launch
    models
    world
DESTINATION share/${PROJECT_NAME}
)
```

---

## minimum launch

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    ld = LaunchDescription()

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg = get_package_share_directory('ign_tutorial')

    resources = [
        os.path.join(pkg, "worlds"),
        os.path.join(pkg, "models")
    ]
    resource_env = SetEnvironmentVariable(name="IGN_GAZEBO_RESOURCE_PATH", value=":".join(resources))

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': '-r -v 2 empty.sdf'}.items(),
    )

    ld.add_action(resource_env)
    ld.add_action(gazebo)
    return ld
```
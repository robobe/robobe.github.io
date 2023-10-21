---
tags:
    - ros2
    - gazebo-classic
    - launch
    - gazebo
---
# Ros2 gazebo classic

```bash
sudo apt install ros-humble-gazebo-ros-pkgs
```

## launch
Using launch file from `gazebo-ros` package

```
ros2 launch gazebo_ros gazebo.launch.py
```

!!! tip ""
    Run Gazebo Server and client `launches` for more control

    - gzclient.launch.py  
    - gzserver.launch.py
     
    from `gzserver.launch.py` for example we can control pause verbose and other argument

    ```
    ros2 launch gazebo_ros gzserver.launch.py -s
    ```

## launch

```python
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from pathlib import Path


def generate_launch_description():
    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name = "bot_description"  # <--- CHANGE ME
    pkg_description = get_package_share_directory(package_name)
    pkg_gazebo = get_package_share_directory("gazebo_ros")
    
    resources = [os.path.join(pkg_description, "worlds")]
    resource_env = AppendEnvironmentVariable(
        name="GAZEBO_RESOURCE_PATH", value=":".join(resources)
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gz_server_launch = (
        Path(pkg_gazebo).joinpath("launch", "gzserver.launch.py").as_posix()
    )
    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gz_server_launch]),
        launch_arguments={
            "pause": "false",
            "verbose": "true",
            "world": "playground.sdf"
        }.items(),
    )

    gz_client_launch = (
        Path(pkg_gazebo).joinpath("launch", "gzclient.launch.py").as_posix()
    )
    gz_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gz_client_launch])
    )


    # Launch them all!
    return LaunchDescription(
        [
            resource_env,
            gz_server,
            gz_client
        ]
    )
```



---

# Reference
- [ROS2 Gazebo migration guide](https://github.com/ros-simulation/gazebo_ros_pkgs/wiki)
- [Installing gazebo_ros_pkgs (ROS 2)](http://classic.gazebosim.org/tutorials?tut=ros2_installing&cat=connect_ros)
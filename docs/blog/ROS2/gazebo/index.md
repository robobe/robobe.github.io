---
tags:
    - ros2
    - gazebo-classic
    - launch
    - gazebo
---
# Ros2 gazebo classic
launch gazebo form launch file

```bash
sudo apt install ros-humble-gazebo-ros-pkgs
```

## launch
Using launch file from `gazebo-ros` package

```
ros2 launch gazebo_ros gazebo.launch.py
```


Run Gazebo **Server** and **client** `launches` for `gazebo_ros` package to get more control

- `gzclient.launch.py`
- `gzserver.launch.py`
     


!!! tip ""
    Show launch arguments
    ```
    ros2 launch gazebo_ros gzserver.launch.py -s
    ```

## launch

```python
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from pathlib import Path


def generate_launch_description():
    pkg_gazebo = get_package_share_directory("gazebo_ros")

    # GzServer
    gz_server_launch = (
        Path(pkg_gazebo).joinpath("launch", "gzserver.launch.py").as_posix()
    )
    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gz_server_launch]),
        launch_arguments={
            "pause": "false",
            "verbose": "true"
        }.items(),
    )

    #GzClient
    gz_client_launch = (
        Path(pkg_gazebo).joinpath("launch", "gzclient.launch.py").as_posix()
    )
    gz_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gz_client_launch])
    )

    return LaunchDescription(
        [
            gz_server,
            gz_client
        ]
    )
```

---

## gazebo environment variables
[](https://github.com/ros-simulation/gazebo_ros_pkgs/wiki/ROS-2-Migration:-Gazebo-ROS-Paths)

---

# Reference
- [ROS2 Gazebo migration guide](https://github.com/ros-simulation/gazebo_ros_pkgs/wiki)
- [Installing gazebo_ros_pkgs (ROS 2)](http://classic.gazebosim.org/tutorials?tut=ros2_installing&cat=connect_ros)
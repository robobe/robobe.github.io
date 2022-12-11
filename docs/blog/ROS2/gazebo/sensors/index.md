---
title: Gazebo (classic) sensors tutorials       
tags:
    - gazebo
    - ros2
    - cook
---


## Project

```
gazebo
    ├── worlds
    │   └── gazebo.world
    ├── models
    │   └── simple
    ├── launch
    │   └── gazebo.launch.py
```

---

## launch

```python title="" linenums="1" hl_lines="1"
import os
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    world = "gazebo.world"

    resources = [
        "/home/user/nav2_ws/src/navigation2_tutorials/sam_bot_description/worlds"
    ]
    resource_env = AppendEnvironmentVariable(name="GAZEBO_RESOURCE_PATH", value=":".join(resources))

    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={
            "verbose": "true", 
            'world': world}.items())

    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')))
        
    ld = LaunchDescription()
    ld.add_action(resource_env)
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    return ld
```

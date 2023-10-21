---
title: ROS2 RVIZ2
tags:
    - rviz
    - rviz2
    - launch
    - ros2
---
# RVIZ

## Run from cli
```bash
rviz -d <path to config file>
```

## Run from launch

!!! note 
    Rviz conifg file locate at package `config` folder
    don't forget to include `config` folder in `setup` `data_files` argument
    ```
    data_files=[
        (os.path.join('share', package_name, "config"), glob('config/*.rviz'))
    ]
    ```
     
### Launch 


```python title="launch.py"
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from pathlib import Path



def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use sim time if true"
    )
    # Process the URDF file
    pkg_description = get_package_share_directory("bot_description")
    rviz_config = Path(pkg_description).joinpath("config", "rviz.rviz").as_posix()

    rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', rviz_config],
       parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription(
            [
                use_sim_time_arg,
                rviz
            ]
        )

```




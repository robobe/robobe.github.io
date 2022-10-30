---
title: Multiple launch files
tags:
    - ros2
    - launch
---

# LAB
- Run/Include sub launch file
- Pass argument and substitute


### Child launch file
- Run turtlesim node
- Change background color (red channel)

!!! tip "Run child with argument from cli"
    ```bash
    ros2 launch launch_tutorial child.launch.py new_background_r:=0
    ```


### Parent launch file
- Include childe launch
- Pass arguments



## Parent

```python
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution


def generate_launch_description():
    colors = {
        'background_r': '0'
    }

    ld =  LaunchDescription()
    
    include = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('launch_tutorial'),
                    'child.launch.py'
                ])
            ]),
            launch_arguments={
                'new_background_r': TextSubstitution(text=str(colors['background_r']))
            }.items()
        )
        
    ld.add_action(include)
    return ld
```
     
## Child

```python title="child.launch.py"
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    new_background_r = LaunchConfiguration('new_background_r')

    new_background_r_launch_arg = DeclareLaunchArgument(
        'new_background_r',
        default_value='255'
    )


    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='sim'
    )

    change_background_r = ExecuteProcess(
        cmd=[[
            'ros2 param set '
            '/sim background_r ',
            new_background_r
        ]],
        shell=True
    )

    return LaunchDescription([
        new_background_r_launch_arg,
        turtlesim_node,
        change_background_r
    ])
```

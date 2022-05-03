---
title: Part2 - spawn
description: launch gazebo and spawn entity
date: "2022-05-02"
banner: ../ros2.png
tags:
    - gazebo
    - launch
    - spawn
---

```python title="aa" linenums="1" hl_lines="1"
import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from  launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")
    pkg_skbot_gazebo = get_package_share_directory("skbot_gazebo")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, "launch", "gazebo.launch.py"))
    )

    world_arg = DeclareLaunchArgument("world",
            default_value=[os.path.join(pkg_skbot_gazebo, "worlds", "skbot.world"), ""],
            description="hello skbot world"
            )

    verbose_arg = DeclareLaunchArgument("verbose",
            default_value=["true"],
            description="verbose log"
            )

    skbot_description = get_package_share_directory("skbot_description")

    robot_description_path =  os.path.join(
        skbot_description,
        "urdf",
        "skbot.xacro",
    )

    urdf_path =  os.path.join(
        skbot_description,
        "urdf",
        "skbot.urdf",
    )

    doc = xacro.process_file(robot_description_path).toxml()
    out = xacro.open_output(urdf_path)
    out.write(doc)

    spawn_entity = Node(package='gazebo_ros', node_executable='spawn_entity.py',
                        arguments=['-entity', 'demo', '-file', urdf_path],
                        output='screen')

    return LaunchDescription(
        [
            world_arg,
            verbose_arg,
            gazebo,
            spawn_entity
        ]
    )

```     
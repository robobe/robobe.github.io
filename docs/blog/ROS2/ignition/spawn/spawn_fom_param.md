---
title: Spawn model from param
tags:
    - gazebo
    - ign
    - spawn
---

# LAB
Spawn sdf/xacro model into simulation
using `param` argument

```python title="launch/from_param.launch.py" linenums="1" hl_lines="54 55"
import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

WORLD_NAME = "demo"
MODEL_NAME = "sam"

def generate_launch_description():
    package_name = "sam_bot_description"
    sdf_model_path = MODEL_NAME + "/model.sdf"
    world_file_path = WORLD_NAME + ".sdf"

    # Pose where we want to spawn the robot
    spawn_x_val = "-2.0"
    spawn_y_val = "0.0"
    spawn_z_val = "0.5"
    spawn_yaw_val = "0.0"

    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg_share = get_package_share_directory(package_name)
    world_path = os.path.join(pkg_share, "worlds", world_file_path)
    sdf_models_path = os.path.join(pkg_share, "models", sdf_model_path)


    robot_description = open(sdf_models_path).read()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": f"-r {world_path}"}.items(),
    )
    # Launch the robot
    spawn_entity_cmd = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-world",
            WORLD_NAME,
            "-param",
            "robot_description",
            "-x",
            spawn_x_val,
            "-y",
            spawn_y_val,
            "-z",
            spawn_z_val,
            "-Y",
            spawn_yaw_val,
        ],
        parameters=[
            {"robot_description": robot_description},
        ],
        output="screen",
    )

    ld = LaunchDescription()
    ld.add_action(gazebo)
    ld.add_action(spawn_entity_cmd)
    return ld

```
---
title: Spawn gazebo with rviz and bridge
tags:
    - ignition
    - gazebo
    - spawn
---

# LAB
- Spawn sdf/xacro model 
- Run Bridge (bridge clock, joint_states)
- Run Rviz 


```python title="" linenums="1" hl_lines="30 74 82"
import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

WORLD_NAME = "demo"
MODEL_NAME = "sam"


def generate_launch_description():
    package_name = "sam_bot_description"
    sdf_model_path = MODEL_NAME + "/model.sdf"
    world_file_path = WORLD_NAME + ".sdf"
    prefix = f"/world/{WORLD_NAME}/model/{MODEL_NAME}"

    # Pose where we want to spawn the robot
    spawn_x_val = "-2.0"
    spawn_y_val = "0.0"
    spawn_z_val = "0.5"
    spawn_yaw_val = "0.0"

    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg_share = get_package_share_directory(package_name)
    world_path = os.path.join(pkg_share, "worlds", world_file_path)
    sdf_models_path = os.path.join(pkg_share, "models", sdf_model_path)

    sdf_file_content = xacro.process_file(sdf_models_path)

    robot_description = sdf_file_content.toxml()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"robot_description": robot_description},
        ],
    )

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

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(pkg_share, "config", "rviz.rviz")],
        parameters=[{"robot_description": robot_description}],
    )

    # Ign Bridge
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            prefix + "/joint_state@sensor_msgs/msg/JointState@ignition.msgs.Model",
            "/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock",
        ],
        remappings=[(prefix + "/joint_state", "joint_states")],
        output="screen",
    )

    ld = LaunchDescription()
    ld.add_action(robot_state_publisher)
    ld.add_action(gazebo)
    ld.add_action(spawn_entity_cmd)
    ld.add_action(rviz)
    ld.add_action(bridge)
    return ld
```

!!! note "remapping"
    map ignition model `joint state` to `joint_states` ros2 topic

    ```python
        bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            prefix + "/joint_state@sensor_msgs/msg/JointState@ignition.msgs.Model",
            "/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock",
        ],
        remappings=[(prefix + "/joint_state", "joint_states")],
        output="screen",
    )
    ```
     
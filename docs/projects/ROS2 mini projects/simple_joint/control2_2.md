---
title: Robot and control2
tags:
    - control
    - gazebo_ros2_control
    - gazebo
    - position_controller
---

Run controllers from launch file


## load controllers from launch file

- Spawn controller with `controller_manager`
  - Spawn `joint_state_broadcaster` 
  - Spawn `position_controller`
- Register to OnProcessExit event to load controller after controller
  - `join_states` load after robot_spawn
  - `position controller` load after `joint_states`

!!! tip controllers name
    Name set by `YAML` file
     
#### controller manager

```python
joint_state_broadcaster_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=[
        "joint_state_broadcaster",
        "--controller-manager",
        "/controller_manager",
    ],
)

robot_controller_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=[
        "position_controller", 
        "--controller-manager",
        "/controller_manager"
    ],
)
```

#### events

```python
spawn_entity_event = RegisterEventHandler(
    event_handler=OnProcessExit(
        target_action=spawn_entity,
        on_exit=[joint_state_broadcaster_spawner],
    )
)

joint_state_event = RegisterEventHandler(
    event_handler=OnProcessExit(
        target_action=joint_state_broadcaster_spawner,
        on_exit=[robot_controller_spawner],
    )
)
```

![](images/position_gazebo_rviz_cli.png)


## source

```python title="control_v2.launch.py"
from launch import LaunchDescription
import os
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import (AppendEnvironmentVariable, 
    DeclareLaunchArgument,
    RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro

PACKAGE = "simple_joint"
WORLD = "empty.world"
URDF = "robot_v2.urdf"

def generate_launch_description():
    ld = LaunchDescription()

    pkg = get_package_share_directory(PACKAGE)
    gazebo_pkg = get_package_share_directory('gazebo_ros')

    verbose = LaunchConfiguration("verbose")
    arg_gazebo_verbose = DeclareLaunchArgument("verbose", default_value="true")
    world = LaunchConfiguration("world")
    arg_gazebo_world = DeclareLaunchArgument("world", default_value=WORLD)


    resources = [os.path.join(pkg, "worlds")]

    resource_env = AppendEnvironmentVariable(
        name="GAZEBO_RESOURCE_PATH", value=":".join(resources)
    )

    sim_time = LaunchConfiguration("sim_time")
    arg_sim_time = DeclareLaunchArgument("sim_time", default_value="true")

    robot_description_path = os.path.join(pkg, "urdf", URDF)
    doc = xacro.parse(open(robot_description_path))
    xacro.process_doc(doc)
    robot_description = doc.toxml()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"use_sim_time": sim_time, "robot_description": robot_description}],
    )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    gazebo_pkg, 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'verbose': verbose, "world": world}.items()
             )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "demo", "-topic", "robot_description", "-z", "0.0"],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "position_controller", 
            "--controller-manager",
            "/controller_manager"
        ],
    )

    spawn_entity_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    joint_state_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    ld.add_action(resource_env)
    ld.add_action(arg_gazebo_verbose)
    ld.add_action(arg_gazebo_world)
    ld.add_action(arg_sim_time)
    ld.add_action(robot_state_publisher)
    ld.add_action(gazebo)
    ld.add_action(spawn_entity_event)
    ld.add_action(joint_state_event)
    ld.add_action(spawn_entity)
    return ld
```
---
title: Control2 joint effort
tags:
    - control
    - gazebo_ros2_control
    - effort_controller
---

Load and use effort controller

#### launch: load controllers
```python title="launch"
robot_controller_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=[
        "joint_effort_controller", 
        "--controller-manager",
        "/controller_manager"
    ],
)
```

#### urdf control section
```xml title="urdf"
<ros2_control name="GazeboSystem" type="system">
    <hardware>
        <plugin>gazebo_ros2_control/GazeboSystem</plugin>
    </hardware>
    <joint name="base_to_second_joint">
        <command_interface name="effort"/>
        <state_interface name="position"/>
        <state_interface name="velocity"/>
        <state_interface name="effort"/>
    </joint>
</ros2_control>

<gazebo>
    <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
        <parameters>$(find simple_joint)/config/effort.yaml</parameters>
    </plugin>
</gazebo>
```


---

### usage
```bash
ros2 topic list
..
/joint_effort_controller/commands
/joint_effort_controller/transition_event
/joint_state_broadcaster/transition_event
/joint_states
/parameter_events
/performance_metrics
/robot_description
/rosout
/tf
/tf_static

# more info about the controller
ros2 topic info /joint_effort_controller/commands
Type: std_msgs/msg/Float64MultiArray
Publisher count: 0
Subscription count: 1

```

```
ros2 topic pub --once \
/joint_effort_controller/commands  \
std_msgs/msg/Float64MultiArray \
"{data: [1.1]}"
```


![type:video](images/control_effort.webm)

---

### source

#### yaml config file
```yaml title="effort.yaml"
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    joint_effort_controller:
      type: effort_controllers/JointGroupEffortController

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

joint_effort_controller:
  ros__parameters:
    joints:
      - base_to_second_joint
    interface_name: effort
    command_interfaces:
      - effort
    state_interfaces:
      - position
      - velocity
```

#### robot.urdf
```xml
<?xml version="1.0"?>
<robot name="simple_example">
    <link name="world"/>

    <material name="blue">
        <color rgba="0 0 0.8 1"/>
    </material>

    <material name="black">
        <color rgba="0 0 0 1"/>
    </material>

    <joint name="word2base" type="fixed">
        <parent link="world"/>
        <child link="base_link"/>
        <origin xyz="0.0 0.0 0.12" rpy="0.0 0.0 0.0"/>
    </joint>

    <link name="base_link">
        <inertial>
            <mass value="10" />
            <inertia ixx="0.4" ixy="0.0" ixz="0.0" iyy="0.4" iyz="0.0" izz="0.2"/>
        </inertial>
        <collision>
            <geometry>
                <cylinder radius="0.05" length="0.24" />
            </geometry>
        </collision>
        <visual>
            <geometry>
                <cylinder radius="0.05" length="0.24" />
            </geometry>
            <material name="black"/>
        </visual>
    </link>

    <link name="second_link">
        <inertial>
            <mass value="0.18" />
            <inertia ixx="0.0002835" ixy="0.0" ixz="0.0" iyy="0.0002835" iyz="0.0" izz="0.000324" />
        </inertial>
        <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0" />
        <collision>
            <geometry>
                <box size="0.15 0.05 0.05" />
            </geometry>
        </collision>
        <visual>
            <geometry>
                <box size="0.15 0.05 0.05" />
            </geometry>
            <material name="blue"/>
        </visual>
    </link>

    <joint name="base_to_second_joint" type="continuous">
        <parent link="base_link"/>
        <child link="second_link"/>
        <axis xyz="0 0 1"/>
        <origin xyz="0.0 0.0 0.2" rpy="0.0 0.0 0.0"/>
        <dynamics damping="0.1" friction="1"/>
    </joint>

    <gazebo reference="base_link">
        <material>Gazebo/Black</material>
    </gazebo>
    <gazebo reference="second_link">
        <material>Gazebo/Red</material>
    </gazebo>

    <!-- Position Config -->
    <ros2_control name="GazeboSystem" type="system">
        <hardware>
            <plugin>gazebo_ros2_control/GazeboSystem</plugin>
        </hardware>
        <joint name="base_to_second_joint">
            <command_interface name="effort"/>
            <state_interface name="position"/>
            <state_interface name="velocity"/>
            <state_interface name="effort"/>
        </joint>
    </ros2_control>

    <gazebo>
        <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
            <parameters>$(find simple_joint)/config/effort.yaml</parameters>
        </plugin>
    </gazebo>
</robot>
```

#### launch
```python
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
URDF = "robot_effort.urdf"

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
            "joint_effort_controller", 
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


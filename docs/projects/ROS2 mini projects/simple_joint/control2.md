---
title: Robot and control2
tags:
    - control
    - gazebo_ros2_control
    - gazebo
    - position_controller
---

Using ros_control2 to control joint position
- Using gazebo hardware
- Add gazebo control section and plugin
- Launch and run
  - position command
  - get joint status

## Gazebo control hardware and plugin
### config hardware
- load gazebo hardware
- config joint command and state
  
```xml
<ros2_control name="GazeboSystem" type="system">
    <hardware>
        <plugin>gazebo_ros2_control/GazeboSystem</plugin>
    </hardware>
    <joint name="base_to_second_joint">
        <command_interface name="position"/>
        <state_interface name="position"/>
        <state_interface name="velocity"/>
        <state_interface name="effort"/>
    </joint>
</ros2_control>
```

### plugin 
```xml
<gazebo>
    <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
        <parameters>$(find simple_joint)/config/position.yaml</parameters>
    </plugin>
</gazebo>
```

#### plugin yaml
```yaml title="position.yaml"
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    position_controller:
      type: position_controllers/JointGroupPositionController

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

position_controller:
  ros__parameters:
    joints:
      - base_to_second_joint
    interface_name: position
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity

```

---

### Demo
- Launch gazebo
- Load controllers
- Set position command
- Get state


### gazebo log
- load plugin
- Init hardware
- Loading controller manager
  
```
[gazebo_ros2_control]: Loading gazebo_ros2_control plugin
[gazebo_ros2_control]: Starting gazebo_ros2_control plugin in namespace: /
[gazebo_ros2_control]: Starting gazebo_ros2_control plugin in ros 2 node: gazebo_ros2_control
[gazebo_ros2_control]: Loading parameter file /home/user/ros2_ws/install/simple_joint/share/simple_joint/config/position.yaml
[gazebo_ros2_control]: connected to service!! robot_state_publisher
[gazebo_ros2_control]: Recieved urdf from param server, parsing...
[gazebo_ros2_control]: Loading joint: base_to_second_joint
[gazebo_ros2_control]: 	State:
[gazebo_ros2_control]: 		 position
[gazebo_ros2_control]: 		 velocity
[gazebo_ros2_control]: 		 effort
[gazebo_ros2_control]: 	Command:
[gazebo_ros2_control]: 		 position
[resource_manager]: Initialize hardware 'GazeboSystem' 
[resource_manager]: Successful initialization of hardware 'GazeboSystem'
[resource_manager]: 'configure' hardware 'GazeboSystem' 
[resource_manager]: Successful 'configure' of hardware 'GazeboSystem'
[resource_manager]: 'activate' hardware 'GazeboSystem' 
[resource_manager]: Successful 'activate' of hardware 'GazeboSystem'
[gazebo_ros2_control]: Loading controller_manager
[gazebo_ros2_control]:  Desired controller update period (0.01 s) is slower than the gazebo simulation period (0.001 s).
[gazebo_ros2_control]: Loaded gazebo_ros2_control.
```

## load controllers
### joint state controller
```
ros2 run controller_manager spawner joint_state_broadcaster
```

### position controller
```
ros2 run controller_manager spawner position_controller
```

## usage

```bash
ros2 control list_controllers
position_controller [position_controllers/JointGroupPositionController] active    
joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
```

```bash title="" linenums="1" hl_lines="4 6"
ros2 topic list
#
/joint_state_broadcaster/transition_event
/joint_states
...
/position_controller/commands
/position_controller/transition_event
/robot_description
/rosout
/tf
/tf_static

```

### position_controller/command
```bash
ros2 topic info /position_controller/commands 
Type: std_msgs/msg/Float64MultiArray
Publisher count: 0
Subscription count: 1
```

```bash
ros2 topic pub --once \
/position_controller/commands  \
std_msgs/msg/Float64MultiArray \
"{data: [1.57]}"
```

### joint_states

```bash
ros2 topic echo --once /joint_states
A message was lost!!!
	total count change:1
	total count: 1---
header:
  stamp:
    sec: 776
    nanosec: 550000000
  frame_id: ''
name:
- base_to_second_joint
position:
- 1.5700000000000003
velocity:
- 0.0
effort:
- 0.0
---
```
![](images/gazebo_position_control_90.png)

## source
### urdf

```xml title="robot_v2.urdf
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
            <command_interface name="position"/>
            <state_interface name="position"/>
            <state_interface name="velocity"/>
            <state_interface name="effort"/>
        </joint>
    </ros2_control>

    <gazebo>
        <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
            <parameters>$(find simple_joint)/config/position.yaml</parameters>
            <!-- <parameters>$(find simple_joint)/config/velocity.yaml</parameters> -->
            <!-- <parameters>$(find simple_joint)/config/effort.yaml</parameters> -->
        </plugin>
    </gazebo>
</robot>
```

### launch
```python title="position_control.launch.py"
from launch import LaunchDescription
import os
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument
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

    ld.add_action(resource_env)
    ld.add_action(arg_gazebo_verbose)
    ld.add_action(arg_gazebo_world)
    ld.add_action(arg_sim_time)
    ld.add_action(robot_state_publisher)
    ld.add_action(gazebo)
    ld.add_action(spawn_entity)
    return ld
```


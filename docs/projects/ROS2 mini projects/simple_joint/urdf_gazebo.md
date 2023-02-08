---
title: Simple joint gazebo simulation
tags:
    - urdf
    - gazebo
    - launch
    - gazebo_ros_force
    - gazebo_ros_joint_state_publisher
---
launch gazebo and spawn our `simple_joint` robot
- apply torque to link from gazebo gui
- Add `gazebo_force` plugin
- apply torque from command line

## launch gazebo
### urdf
```xml title="robot_v1.urdf|
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
                <box size="0.05 0.05 0.15" />
            </geometry>
        </collision>
        <visual>
            <geometry>
                <box size="0.05 0.05 0.15" />
            </geometry>
            <material name="blue"/>
        </visual>
    </link>

    <joint name="base_to_second_joint" type="continuous">
        <parent link="base_link"/>
        <child link="second_link"/>
        <axis xyz="0 0 1"/>
        <origin xyz="0.0 0.0 0.2" rpy="0.0 0.0 0.0"/>
    </joint>

    <gazebo>
        <plugin name="gazebo_ros_joint_state_publisher" filename="libgazebo_ros_joint_state_publisher.so">
            <update_rate>50</update_rate>
            <joint_name>base_to_second_joint</joint_name>
        </plugin>
    </gazebo>

    <gazebo reference="base_link">
        <material>Gazebo/Black</material>
    </gazebo>
    <gazebo reference="second_link">
        <material>Gazebo/Red</material>
    </gazebo>
</robot>
```

### launch
```python title="gz_sim.launch.py"
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
URDF = "robot_v1.urdf"

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

---

## Apply torque
- Apply torque to joint
  
![](images/gazebo_simple_joint.png)

!!! note "gazebo joint state plugin"
    ```
    <gazebo>
        <plugin name="gazebo_ros_joint_state_publisher" filename="libgazebo_ros_joint_state_publisher.so">
            <update_rate>50</update_rate>
            <joint_name>base_to_second_joint</joint_name>
        </plugin>
    </gazebo>
    ```
     

![type:video](images/gazebo_rviz_joint_publisher.webm)



---

## Apply Force/ Torque using Wrench msg 
Create topic with `Wrench` message


!!! note Add damping and friction
     ```xml
     <joint name="base_to_second_joint" type="continuous">
        <parent link="base_link"/>
        <child link="second_link"/>
        <axis xyz="0 0 1"/>
        <origin xyz="0.0 0.0 0.2" rpy="0.0 0.0 0.0"/>
        <dynamics damping="0.1" friction="1"/>
    </joint>
     ```

```xml
<gazebo>
    <plugin name="gazebo_ros_force" filename="libgazebo_ros_force.so">
        <link_name>second_link</link_name>
        <force_frame>world</force_frame>
    </plugin>
</gazebo>
```

### Wrench msg
```
ros2 topic pub --once /gazebo_ros_force geometry_msgs/msg/Wrench "{force: {x: 0.0, y: 0.0, z: 0.0}, torque: {x: 0.0,y: 0.0,z: 1.1}}"
```

---

# Reference 
- [urdf joint](http://wiki.ros.org/urdf/XML/joint)
- [gazebo-ros-force](https://github.com/ros-simulation/gazebo_ros_pkgs/wiki/ROS-2-Migration:-Force)


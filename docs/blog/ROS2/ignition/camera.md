---
title: Ignition ROS2 camera bridge
tags:
    - ignition
    - ros2
    - camera
    - bridge
---

- IGNITION: fortress
- ROS2: humble


## Objective
- Create simulation with camera
- Publish camera using ros_ign bridge
- Write launch file
- View camera image in RVIZ


## model with camera

- Based on vehicle model from [this post](joint_state_sdf_model.md)
- Add this lines to exists model

!!! tip "don't forget"
     World sdf must contain `sensors` plugin

     ```xml
     <plugin filename="ignition-gazebo-sensors-system" name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre</render_engine>
    </plugin>
     ```

```xml
<frame name="camera_frame" attached_to='chassis'>
    <pose>-0.8 0 1.5 0 0 0</pose>
</frame>

<link name="camera_link">
    <pose relative_to='camera_frame'>0 0 0 0 0 0</pose>
    <inertial>
    <mass>0.1</mass>
    <inertia>
        <ixx>0.000166667</ixx>
        <iyy>0.000166667</iyy>
        <izz>0.000166667</izz>
    </inertia>
    </inertial>
    <collision name="collision">
    <geometry>
        <box>
        <size>0.1 0.1 0.1</size>
        </box>
    </geometry>
    </collision>
    <visual name="visual">
    <geometry>
        <box>
        <size>0.1 0.1 0.1</size>
        </box>
    </geometry>
    </visual>
    <sensor name="camera" type="camera">
    <camera>
        <horizontal_fov>1.047</horizontal_fov>
        <image>
        <width>320</width>
        <height>240</height>
        </image>
        <clip>
        <near>0.1</near>
        <far>100</far>
        </clip>
    </camera>
    <always_on>1</always_on>
    <update_rate>30</update_rate>
    <visualize>true</visualize>
    <topic>camera</topic>
    </sensor>
</link>

<joint name='camera_joint' type='fixed'>
    <parent>chassis</parent>
    <child>camera_link</child>
</joint>

```

## launch

```python
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

PACKAGE_NAME = "ign_tutorial"

def generate_launch_description():
    pkg = get_package_share_directory(PACKAGE_NAME)
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    paths = [
     os.path.join(pkg, "worlds"),
     "/home/user/wasp_ws/src/tutorials/ign_tutorial/models"
    ]
    env = SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH', value=[":".join(paths)])

    sdf_file = os.path.join(pkg, 'models', 'vehicle', 'model.sdf')

    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': '-r -v 4 my_cart.sdf'
        }.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},
        ]
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/world/Moving_robot/model/vehicle/model/vehicle_blue/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/camera@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'
        ],
        remappings=[
            ('/world/Moving_robot/model/vehicle/model/vehicle_blue/joint_state', 'joint_states'),
        ],
        output='screen'
    )

    rviz_node = Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d' + os.path.join(pkg, 'config', 'rviz.rviz')]
        )

    world_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name="world2chassis",
        arguments = ["0", "0", "0", "0", "0", "0", "world", "chassis"]
    )

    return LaunchDescription([
        env,
        gz_sim,
        bridge,
        world_tf,
        robot_state_publisher,
        rviz_node  
    ])
```

## Rviz
- [Rviz types](http://wiki.ros.org/rviz/DisplayTypes)

| type  | Desc  |
|---|---|
| Camera  | use `CameraInfo` to create window in show the image  |
| Image  | display image without `CameraInfo` data  |

![](image/rviz_with_camera.png)

---

## ign

![](image/ign_with_image_plugin.png)
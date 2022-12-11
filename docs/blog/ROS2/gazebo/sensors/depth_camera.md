---
title: ROS2 gazebo (classic) depth camera sensor
tags:
    - gazebo
    - ros2
    - sensors
    - cook
---

# sdf

```xml title="model" linenums="1" hl_lines="1"
<?xml version="1.0"?>
<sdf version="1.5">
    <model name="depth_camera">
        <pose>0 0 0.5 0 0 0</pose>
        <static>true</static>
        <link name="camera_depth_frame">
            <pose>0 0 0 -1.5708 0 -1.5708</pose>
          </link>
        <link name="link">
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
            <sensor type="depth" name="depth_camera">
                <always_on>0</always_on>
                <update_rate>10</update_rate>
                <camera name="camera_name">
                </camera>
                <plugin name="plugin_name" filename="libgazebo_ros_camera.so">
                    <ros>
                        <namespace>custom_ns</namespace>
                        <remapping>custom_camera/image_raw:=custom_camera/custom_image</remapping>
                        <remapping>custom_camera/image_depth:=custom_camera/custom_image_depth</remapping>
                        <remapping>custom_camera/camera_info:=custom_camera/custom_info_raw</remapping>
                        <remapping>custom_camera/camera_info_depth:=custom_camera/custom_info_depth</remapping>
                        <remapping>custom_camera/points:=custom_camera/custom_points</remapping>
                    </ros>
                    <camera_name>custom_camera</camera_name>
                    <frame_name>camera_depth_frame</frame_name>
                </plugin>
            </sensor>
        </link>
    </model>
</sdf>
```
---

# launch

```python title="launch" linenums="1" hl_lines="1"
import os
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

PACKAGE_NAME = "sam_bot_description"
WORLD = "gazebo.world"
MODEL = "depth_camera"

def generate_launch_description():
    pkg_share = get_package_share_directory(PACKAGE_NAME)
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    model_sdf_full_path = os.path.join(pkg_share, "models", MODEL, "model.sdf")

    
    resources = [
        os.path.join(pkg_share, "worlds")    
    ]

    resource_env = AppendEnvironmentVariable(name="GAZEBO_RESOURCE_PATH", value=":".join(resources))

    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={
            "verbose": "true", 
            'world': WORLD}.items())

    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')))
        
    spawn_entity_cmd = Node(
        package="gazebo_ros", 
        executable="spawn_entity.py",
        arguments=['-entity', "robot_name_in_model", 
        '-file', model_sdf_full_path,
        '-x', "0",
        '-y', "0",
        '-z', "0.5"],
        output='screen')

    ld = LaunchDescription()
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(pkg_share, "config", MODEL + ".rviz")],
    )

    link_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name="link2world",
        arguments = ["0", "0", "0.5", "-1.5708", "0", "-1.5708", "link", "camera_depth_frame"]
    )

    ld = LaunchDescription()
    ld.add_action(resource_env)
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(spawn_entity_cmd)
    ld.add_action(rviz)
    ld.add_action(link_tf)
    return ld
``` 
---

# Run

![](images/depth_camera.png)

---

# Reference
- [ROS2 migration](https://github.com/ros-simulation/gazebo_ros_pkgs/wiki/ROS-2-Migration:-Camera#gazebo_ros_depth_camera)
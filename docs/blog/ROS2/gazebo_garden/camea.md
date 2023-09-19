---
tags:
    - gz
    - gazebo
    - camera
    - sdf
---
# Camera sensor
[sdf specification](http://sdformat.org/spec?ver=1.10&elem=sensor#sensor_camera)

## SDF
- Camera model
  - Link
  - Sensor


```xml title="camera model"
<model name="camera">
    <static>true</static>
    <pose>0 0 1.0 0 0 0</pose>
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
        <update_rate>20</update_rate>
        <topic>camera</topic>
        <always_on>1</always_on>
        <visualize>true</visualize>
        </sensor>
    </link>
    </model>
```

## Gazebo
- Add sensor plugin
- Add Camera model
- Add ImageDisplay GUI plugin

```xml
<plugin title="Sensors plugin"
      filename="ignition-gazebo-sensors-system"
      name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre</render_engine>
</plugin>
```

!!! note "render engine"
    TODO: what is the default, why is matter
    orge
    orge2


```xml title="image display"
<plugin filename="ImageDisplay" name="Image Display">
  <ignition-gui>
    <property key="state" type="string">docked</property>
  </ignition-gui>
</plugin>
```     

---

## ROS2

### Bridge
#### parameter_bridge

```bash title="bridge"
ros2 run ros_gz_bridge parameter_bridge "/camera@sensor_msgs/msg/Image[gz.msgs.Image"
```

!!! tip "Reminder"
    if zsh using as shell surround the mapping with quot
!!! tip "Reminder"
    Bridge message direction

    - **@**  == a bidirectional bridge, 
    - **[**  == a bridge from Gazebo to ROS,
    - **]**  == a bridge from ROS to Gazebo.

     
```bash title="topics"
ros2 topic list
#
/camera
...
```

```bash title="image view"
ros2 run rqt_image_view rqt_image_view /camera
```

![](image/rqt_image_view_camera_topic.png)


### Using YAML

```yaml
- topic_name: /camera
  ros_type_name: sensor_msgs/msg/Image
  gz_type_name: gz.msgs.Image
  direction: GZ_TO_ROS
```

!!! tip "direction"
    [ros_gz_bridge README](https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_bridge/README.md#example-5-configuring-the-bridge-via-yaml)

    - **BIDIRECTIONAL** (Default) - Bridge both directions
    - **GZ_TO_ROS** - Bridge Ignition topic to ROS
    - **ROS_TO_GZ** - Bridge ROS topic to Ignition

```bash title="run bridge with yaml config"
ros2 run ros_gz_bridge \
parameter_bridge \
--ros-args -p config_file:=/home/user/gz_ws/src/gz_demos/config/ign2ros_bridge.yaml
```

### launch

```python title="image_bridge.launch.py"
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # Bridge
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/camera@sensor_msgs/msg/Image[gz.msgs.Image",
        ],
        output="screen",
    )

    viewer = Node(
        package="rqt_image_view",
        executable="rqt_image_view",
        name="view",
        arguments=[
            "/camera"
        ]
    )

    ld.add_action(bridge)
    ld.add_action(viewer)
    return ld

```
---

#### image_bridge

```bash title="image_bridge"
ros2 run ros_gz_image image_bridge /camera
```

```bash title="topics"
ros2 topic list
#
/camera
/camera/compressed
/camera/compressedDepth
/camera/theora

...
```

---

## world

```xml title="camera_world.sdf"
<?xml version="1.0" ?>
<sdf version="1.8" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
 xsi:noNamespaceSchemaLocation="http://sdformat.org/schemas/root.xsd" >
  <world name="camera">
    <physics name="fast" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
    </physics>
    <plugin
      filename="gz-sim-physics-system"
      name="gz::sim::systems::Physics">
    </plugin>
    <plugin
      filename="gz-sim-user-commands-system"
      name="gz::sim::systems::UserCommands">
    </plugin>
    <plugin
      filename="gz-sim-scene-broadcaster-system"
      name="gz::sim::systems::SceneBroadcaster">
    </plugin>
    <plugin
      filename="gz-sim-contact-system"
      name="gz::sim::systems::Contact">
    </plugin>

    <plugin
      filename="gz-sim-sensors-system"
      name="gz::sim::systems::Sensors">
      
    </plugin>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <model name="camera">
        <static>true</static>
        <pose>0 0 1.0 0 0 0</pose>
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
            <update_rate>20</update_rate>
            <topic>camera</topic>
            <always_on>1</always_on>
            <visualize>true</visualize>
          </sensor>
        </link>
      </model>

      <model name="box">
        <static>true</static>
        <pose>3 0 0.5 0 0 0</pose>
        <link name="link">
          <collision name="collision">
            <geometry>
              <box>
                <size>1 1 1</size>
              </box>
            </geometry>
          </collision>
          <visual name="visual">
            <geometry>
              <box>
                <size>1 1 1</size>
              </box>
            </geometry>
          </visual>
        </link>
      </model>
  </world>
</sdf>
```
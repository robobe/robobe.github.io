---
tags:
    - ros2
    - gazebo
    - gps
    - sensors
---
# Gazebo GPS Sensor

Add GPS sensor and ROS2 plugin

!!! note 
    GPS gazebo is a sensor plugin

---

## Demo project

```
sdf_lab
├── launch
│   └── gps_world.launch.py
├── models
│   └── gps_box
│      ├── model.config
│      └── model.sdf
├── package.xml
├── setup.cfg
├── setup.py
└── worlds
    └── empty_gps.world
```

Project files: 

- model: Simple link with gps sensor and ros2 plugin  
- world: Init gps coordinates  
- launch: Run gazebo with world and spawn model

### model
```xml
<?xml version="1.0"?>
<sdf version="1.5">
    <model name="gps_box">
        <pose>0 0 0 0 0 0</pose>

        <link name='gps_link'>
            <pose>0 0 0.2 0 0 0</pose>
            <inertial>
                <mass>1</mass>
                <inertia>
                    <ixx>0.026666666666666672</ixx>
                    <ixy>0</ixy>
                    <ixz>0</ixz>
                    <iyy>0.026666666666666672</iyy>
                    <iyz>0</iyz>
                    <izz>0.026666666666666672</izz>
                </inertia>
            </inertial>
            <visual name='box_visual'>
                <geometry>
                    <box>
                        <size>0.4 0.4 0.4</size>
                    </box>
                </geometry>
                <material>
                    <script>
                        <uri>file://media/materials/scripts/gazebo.material</uri>
                        <name>Gazebo/Red</name>
                    </script>
                </material>

            </visual>
            <collision name='box_collision'>
                <geometry>
                    <box>
                        <size>0.4 0.4 0.4</size>
                    </box>
                </geometry>
            </collision>
            <sensor name="gps_sensor" type="gps">
                <always_on>true</always_on>
                <update_rate>10</update_rate>
                <plugin name="gps_controller" filename="libgazebo_ros_gps_sensor.so">
                    <ros>
                        <!-- <namespace>/br</namespace> -->
                        <remapping>gps_controller/out:=gps_pos</remapping>
                        <remapping>gps_controller/vel:=gps_vel</remapping>

                    </ros>
                    <frame_name>gps_link</frame_name>
                </plugin>
            </sensor>
        </link>

  
        
    </model>
</sdf>
```

### world
```xml
<sdf version="1.6">
    <world name="gazebo_ros_gps_sensor_world">
      <include>
        <uri>model://ground_plane</uri>
      </include>
      <include>
        <uri>model://sun</uri>
      </include>
      <spherical_coordinates>
        <latitude_deg>31.0461</latitude_deg>
        <longitude_deg>34.8516</longitude_deg>
        <elevation>0</elevation>
          <!-- currently gazebo has a bug: instead of outputing lat, long, altitude in ENU
          (x = East, y = North and z = Up) as the default configurations, it's outputting (-E)(-N)U,
          therefore we rotate the default frame 180 so that it would go back to ENU -->
          <heading_deg>180</heading_deg>
      </spherical_coordinates>
    </world>
</sdf>
```     

### launch
```python
from launch import LaunchDescription
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

PACKAGE = "sdf_lab"
WORLD = "empty_gps.world"

def generate_launch_description():
    ld = LaunchDescription()

    pkg = get_package_share_directory(PACKAGE)
    gazebo_pkg = get_package_share_directory('gazebo_ros')

    verbose = LaunchConfiguration("verbose")
    arg_gazebo_verbose = DeclareLaunchArgument("verbose", default_value="true")
    world = LaunchConfiguration("world")
    arg_gazebo_world = DeclareLaunchArgument("world", default_value=WORLD)


    resources = ["/usr/share/gazebo-11", os.path.join(pkg, "worlds")]

    resource_env = AppendEnvironmentVariable(
        name="GAZEBO_RESOURCE_PATH", value=":".join(resources)
    )

    models = [os.path.join(pkg, "models")]

    models_env = AppendEnvironmentVariable(
        name="GAZEBO_MODEL_PATH", value=":".join(models)
    )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    gazebo_pkg, 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'verbose': verbose, "world": world}.items()
             )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "demo", "-database", "gps_box"],
        output="screen",
    )

    ld.add_action(models_env)
    ld.add_action(resource_env)
    ld.add_action(arg_gazebo_verbose)
    ld.add_action(arg_gazebo_world)
    ld.add_action(gazebo)
    ld.add_action(spawn_entity)
    return ld

```

!!! tip using spawn database argument
    set model name and set GAZEBO_MODEL_PATH path
    this attribute lock for model by name in this paths
     
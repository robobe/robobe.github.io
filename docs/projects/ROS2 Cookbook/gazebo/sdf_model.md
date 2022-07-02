---
title: Spawn gazebo world with SDF model
tags:
    - gazebo
    - sdf
    - launch
---

Run gazebo with model sdf, set gazebo environment by add tag to `package.xml`
```
<gazebo_ros gazebo_model_path="${prefix}/models" />
```

!!! note 
     `prefix` is substitute with current file location
     `install/rrbot_description/share/rrbot_description/`
     

```bash title="projects"
rrbot_description
    ├── CMakeLists.txt
    ├── package.xml
    └── models
        └── simple_box
            ├── model.config
            └── model.sdf

rrbot_gazebo
├── CMakeLists.txt
├── package.xml
└── launch
    └── sdf_world.launch.py
```

### rrbot_description

#### models/simple_box

```xml title="model.conifg"
<?xml version="1.0"?>
<model>
  <name>simple_box</name>
  <version>1.0</version>
  <sdf version="1.5">model.sdf</sdf>
  <author>
    <name></name>
    <email></email>
  </author>
  <description>
  </description>
</model> 
```

```xml title="model.sdf"
<?xml version='1.0'?>
<sdf version="1.4">
  <model name="simple_box">
    <pose>0 0 0.5 0 0 0</pose>
    <static>true</static>
    <link name="link">
      <inertial>
        <mass>1.0</mass>
        <inertia> <!-- inertias are tricky to compute -->
          <!-- http://gazebosim.org/tutorials?tut=inertia&cat=build_robot -->
          <ixx>0.083</ixx>       <!-- for a box: ixx = 0.083 * mass * (y*y + z*z) -->
          <ixy>0.0</ixy>         <!-- for a box: ixy = 0 -->
          <ixz>0.0</ixz>         <!-- for a box: ixz = 0 -->
          <iyy>0.083</iyy>       <!-- for a box: iyy = 0.083 * mass * (x*x + z*z) -->
          <iyz>0.0</iyz>         <!-- for a box: iyz = 0 -->
          <izz>0.083</izz>       <!-- for a box: izz = 0.083 * mass * (x*x + y*y) -->
        </inertia>
      </inertial>
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
</sdf>
```

---

#### CMakeLists.txt
- Add `install` command to `CMakeLists.txt`
  - Copy `models` folder to destination


```c
install(DIRECTORY
  models
  DESTINATION share/${PROJECT_NAME}
)
```

---

#### package.xml

- Add `gazebo_ros` tag, usage by gazebo.launch file to set gazebo environment variables


```xml linenums="1" hl_lines="3" 
<export>
    <build_type>ament_cmake</build_type>
    <gazebo_ros gazebo_model_path="${prefix}/models" />
</export>
```


---

## rrbot_gazebo

#### worlds

```xml title="sdf.world"
<?xml version="1.0" ?>
<sdf version="1.4">
  <!-- We use a custom world for the rrbot so that the camera angle is launched correctly -->

  <world name="default">
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Global light source -->
    <include>
      <uri>model://sun</uri>
    </include>

    <include>
      <uri>model://simple_box</uri>
    </include>
  </world>
</sdf>
```

---

#### launch
```python title="sdf_world.launch.py" linenums="1" 
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

pkg_description_pkg = "rrbot_description"
package_name = 'rrbot_gazebo'
world_file = 'sdf.world'

def generate_launch_description():

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_simulation = get_package_share_directory(package_name)
    

    # launch Gazebo by including its definition
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        ),
        launch_arguments=[("verbose", "true")]
    )


    # load the world file
    world_arg = DeclareLaunchArgument(
          'world',
          default_value=[os.path.join(pkg_simulation, 'worlds', world_file), ''],
          description='SDF world file')

    return LaunchDescription([
        world_arg,
        gazebo
    ])
```

---

#### CMakeLists.txt
- Add `install` command to `CMakeLists.txt`
  - Copy `worlds` folder to destination


```c
install(DIRECTORY
  worlds
  DESTINATION share/${PROJECT_NAME}
)
```

---

## Build and Run
```bash
# Build
colcon build --symlink-install --packages-select rrbot_description
colcon build --symlink-install --packages-select rrbot_gazebo

# Run
ros2 launch rrbot_gazebo sdf_world.launch.py
```

![](images/sdf_simple_box.png)
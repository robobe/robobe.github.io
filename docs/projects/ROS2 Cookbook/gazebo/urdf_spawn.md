---
title: Spawn gazebo and with urdf
tags:
    - gazebo
    - urdf
    - launch
---

Launch file that start gazebo and spawn `urdf` 

```bash title="projects"
rrbot_description
    ├── CMakeLists.txt
    ├── package.xml
    └── urdf
        └── box.urdf

rrbot_gazebo
├── CMakeLists.txt
├── package.xml
└── launch
    └── spawn_urdf.launch.py
```

### rrbot_description

#### urdf
- Simple box urdf

```xml title="box.urdf"
<robot name="simple_box">
    <link name="my_box">
      <inertial>
        <origin xyz="2 0 0" />
        <mass value="1.0" />
        <inertia  ixx="1.0" ixy="0.0"  ixz="0.0"  iyy="100.0"  iyz="0.0"  izz="1.0" />
      </inertial>
      <visual>
        <origin xyz="2 0 1"/>
        <geometry>
          <box size="1 1 2" />
        </geometry>
      </visual>
      <collision>
        <origin xyz="2 0 1"/>
        <geometry>
          <box size="1 1 2" />
        </geometry>
      </collision>
    </link>
    <gazebo reference="my_box">
      <material>Gazebo/Blue</material>
    </gazebo>
  </robot>
```

#### CMakeLists.txt

- Add `install` command to `CMakeLists.txt`
  - Copy all `urdf` files to destination

```c
install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME}
)
```

---

## rrbot_gazebo
#### CMakeLists.txt
- Add install command copy
  - launch

```c
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
```

### launch
- Start gazebo
  - verbose on
- Spawn urdf using `spawn_entity.py` node from `gazebo_ros` package
- Substitute `gazebo` launch file from `gazebo_ros` package

```python title="spawn_urdf.launch.py" linenums="1" hl_lines="22-26 32"
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

pkg_description_pkg = "rrbot_description"
package_name = 'rrbot_gazebo'

def generate_launch_description():

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_description = get_package_share_directory(pkg_description_pkg)

    urdf_path =  os.path.join(
        pkg_description,
        "urdf",
        "box.urdf",
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        ),
        launch_arguments=[("verbose", "true")]
    )

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-file", urdf_path, "-entity", "robot", "-x", "0.0", "-y", "0.0", "-z", "0.0"],
        output="screen")

    return LaunchDescription([
        gazebo,
        spawn_robot
    ])
```

---

## Build and Run
```bash
# Build
colcon build --symlink-install --packages-select rrbot_description
colcon build --symlink-install --packages-select rrbot_gazebo

# Run
ros2 launch rrbot_gazebo spawn_urdf.launch.py
```

![](images/2022-07-01-21-52-30.png)
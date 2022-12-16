---
title: Part1 - launch gazebo
description: launch gazebo simple example
date: "2022-05-02"
banner: ../ros2.png
tags:
    - gazebo
    - launch
    - 101
---

# project tree

```
skbot_description/
├── CMakeLists.txt
├── config
├── launch
├── meshes
├── package.xml
├── sdf
└── urdf
  ├── skbot.xacro
  ├── macros.xacro
  ├── gazebo.xacro
  └── materials.xacro
```

```
skbot_gazebo/
├── CMakeLists.txt
├── include
├── launch
├── models
├── package.xml
├── src
└── worlds
  └── empty.world
```

---

# gazebo_ros
Provides ROS plugins that offer message and service publishers for interfacing with Gazebo through ROS.

```bash
sudo apt install ros-foxy-gazebo-ros
```

---

## launch empty world

```xml title="empty.world"
<?xml version='1.0'?>
<sdf version="1.6">
<world name="room">
  <include>
    <uri>model://sun</uri>
  </include>
  <include>
    <uri>model://ground_plane</uri>
  </include>
</world>
</sdf>
```

```python title="simple.launch.py"
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

package_name = "skbot_gazebo"
world_file = "empty.world"

def generate_launch_description():

    ld = LaunchDescription()

    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")
    pkg = get_package_share_directory(package_name)

    world = os.path.join(pkg, "worlds", world_file),

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gazebo.launch.py"),
        ),
        launch_arguments={'verbose': "true", 'world': world}.items()
    )

    
    ld.add_action(gazebo)
    
    return ld

```

```cmake title="CMakeLists.txt"
install(DIRECTORY
  models
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY
  worlds
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY
  media
  DESTINATION share/${PROJECT_NAME}/
)
install(TARGETS
  myPlugin
  DESTINATION lib
)
```

### usage
```bash
ros2 launch skbot_gazebo simple.launch.py
```
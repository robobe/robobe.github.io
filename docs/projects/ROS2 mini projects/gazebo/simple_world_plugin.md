---
title: Simple world plugin
tags:
    - gazebo
    - world
    - plugin
---

```bash title="projects"
rrbot_gazebo
    ├── CMakeLists.txt
    ├── launch
    │   └── world_plug.launch.py
    ├── package.xml
    ├── src
    │   └── simple_world_plugin.cpp
    └── worlds
        └── world_plug.world 
```

#### Worlds

```xml title="world_plug.world" linenums="1" hl_lines="14" 
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
    <plugin name="simple" filename="libsimple_world_plugin.so"/>
  </world>
</sdf>
```

---

#### package.xml

```xml linenums="1" hl_lines="3" 
<export>
    <build_type>ament_cmake</build_type>
    <gazebo_ros plugin_path="${prefix}"></gazebo_ros>
</export>
```

---

#### plugin source
```cpp title="simple_world_plugin.cpp"
#include <gazebo/common/Plugin.hh>
#include <rclcpp/rclcpp.hpp>
#include <iostream>

namespace gazebo
{
class WorldPluginTutorial : public WorldPlugin
{
public:
  WorldPluginTutorial() : WorldPlugin()
  {
    // gazebo log
    gzmsg << "gazebo message" << std::endl;
    gzwarn << "gazebo warning" << std::endl;
    gzerr << "gazebo error" << std::endl;

    // ROS log
    RCLCPP_INFO(rclcpp::get_logger("world_plug")," ------ Hello World! ------ ");
    RCLCPP_WARN(rclcpp::get_logger("world_plug")," ------ warning! ------ ");
    RCLCPP_ERROR(rclcpp::get_logger("world_plug")," ------ Error! ------ ");
  }

  void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
  {
  }

};
GZ_REGISTER_WORLD_PLUGIN(WorldPluginTutorial)
}
```

---

#### CMakeLists

```c linenums="1" hl_lines="14-32" 
cmake_minimum_required(VERSION 3.5)
project(rrbot_gazebo)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)

add_library(simple_world_plugin SHARED src/simple_world_plugin.cpp)

target_include_directories(simple_world_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(simple_world_plugin
  "gazebo_ros"
  "rclcpp"
  )

install(TARGETS
simple_world_plugin
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY
  worlds
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

```

#### launch
```python title="world_plug.launch.py" linenums="1" hl_lines="10 28 23" 
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

package_name = 'rrbot_gazebo'
world_file = 'world_plug.world'

def generate_launch_description():

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_simulation = get_package_share_directory(package_name)

    # launch Gazebo by including its definition
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        )
    )
    verbose_arg = DeclareLaunchArgument(
            'verbose', default_value='true',
            description='Set "true" to increase messages written to terminal.'
        )

    # load the world file
    world_arg = DeclareLaunchArgument(
          'world',
          default_value=[os.path.join(pkg_simulation, 'worlds', world_file), ''],
          description='SDF world file')

    return LaunchDescription([
        world_arg,
        verbose_arg,
        gazebo,
        
    ])
```

---

## Usage and Run

```bash
ros2 launch rrbot_gazebo world_plug.launch.py
```

- The first three log line came from gazebo log API
- The Other three came from ROS log API
  
![](images/simple_world_plugin.png)
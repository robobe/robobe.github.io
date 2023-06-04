---
tags:
    - gazebo
    - plugin
    - model
    - ros2
---

Plugins allow complete access to the physical properties of models and their underlying elements (links, joints, collision objects). The following plugin will apply a linear velocity to its parent model.

The following tutorial build and test gazebo module plugin in ROS2 package context

The package include `world` file and `plugin` from `Model plugins` tutorial check reference for links

## Lab
Run gazebo world with model that include custom plugin from ROS launch file

- Create `ament_cmake` package
- Add `launch`,  `plugins`, `worlds` sub folders
- Add `add_subdirectory(plugins)` to root `CMakeLists.txt` file
- Add `install` insruction

```c title="install launch and worlds folder"
install(DIRECTORY 
    launch
    worlds
DESTINATION share/${PROJECT_NAME} )
```


```bash title="project structure"
gazebo_tutorial_pkg
└── src
    ├── CMakeLists.txt
    ├─ gazebo_tutorial_pkg
    ├─ launch
    │       ├── push.launch.py
    ├── plugins
    │       ├── CMakeLists.txt
    │       └── push
    │           └── model_push.cc
    └── worlds
            └── model_push.world
```

#### plugin

```c title="plugin/CMakeLists.txt"
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(model_push SHARED push/model_push.cc)
target_link_libraries(model_push ${GAZEBO_LIBRARIES})

install(TARGETS
model_push
  DESTINATION lib/${PROJECT_NAME}
)
```

```cpp title="plugin/push/model_push.cc"
#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo
{
  class ModelPush : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&ModelPush::OnUpdate, this));
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      // Apply a small linear velocity to the model.
      this->model->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}
```

#### Worlds

```xml title="model_push.world"
<?xml version="1.0"?> 
<sdf version="1.4">
  <world name="default">

    <!-- Ground Plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <include>
      <uri>model://sun</uri>
    </include>

    <model name="box">
      <pose>0 0 0.5 0 0 0</pose>
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

      <plugin name="model_push" filename="libmodel_push.so"/>
    </model>        
  </world>
</sdf>
```

#### launch 

- Set `GAZEBO_RESOURCE_PATH` to share/worlds sub folder
- Set `GAZEBO_PLUGIN_PATH` to lib/<package_name> sub folder


```python title="push.launch.py"
import os

from ament_index_python.packages import (get_package_prefix,
                                         get_package_share_directory)
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

PACKAGE = "gazebo_tutorial_pkg"
WORLD = "model_push.world"

def generate_launch_description():
    ld = LaunchDescription()

    pkg = get_package_share_directory(PACKAGE)
    pkg_root = get_package_prefix(PACKAGE)
    gazebo_pkg = get_package_share_directory("gazebo_ros")

    verbose = LaunchConfiguration("verbose")
    arg_gazebo_verbose = DeclareLaunchArgument("verbose", default_value="true")
    world = LaunchConfiguration("world")
    arg_gazebo_world = DeclareLaunchArgument("world", default_value=WORLD)

    resources = [os.path.join(pkg, "worlds")]
    resource_env = AppendEnvironmentVariable(
        name="GAZEBO_RESOURCE_PATH", value=":".join(resources)
    )

    plugins = [os.path.join(pkg_root, "lib", PACKAGE), os.path.join(pkg, "plugins")]

    plugins_env = AppendEnvironmentVariable(
        name="GAZEBO_PLUGIN_PATH", value=":".join(plugins)
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(gazebo_pkg, "launch", "gazebo.launch.py")]
        ),
        launch_arguments={"verbose": verbose, "world": world}.items(),
    )

    ld.add_action(resource_env)
    ld.add_action(arg_gazebo_verbose)
    ld.add_action(arg_gazebo_world)
    ld.add_action(plugins_env)
    ld.add_action(gazebo)
    return ld

```

---

## Reference
- [gazebo Model plugins](https://classic.gazebosim.org/tutorials?tut=plugins_model)

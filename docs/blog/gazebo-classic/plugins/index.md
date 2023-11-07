---
tags:
    - gazebo classic
    - plugin
---
A plugin is a chunk of code that is compiled as a shared library and inserted into the simulation. The plugin has direct access to all the functionality of Gazebo through the standard C++ classes.

There are currently 6 types of plugins

- World
- Model
- Sensor
- System
- Visual
- GUI

Each plugin type is managed by a different component of Gazebo

!!! note ""
    install gazebo-dev for plugin build
    ```bash
    # ubuntu 22.04
    sudo apt install libgazebo-dev

    # gazebo version
    # sudo apt install libgazebo<XX>-dev
    ``` 

## Reference
- [Plugins 101](https://classic.gazebosim.org/tutorials?tut=plugins_hello_world)

---

## Minimal DEMO
Gazebo world plugin

```
├── bin
├── build
├── CMakeLists.txt
├── src
│   └── world_plugin.cc
└── worlds
    └── demo_world_plugin.world

```

```xml title="demo_world_plugin.world"
<?xml version="1.0"?>
<sdf version="1.4">
  <world name="default">
        <plugin name="hello_world" filename="libhello_world.so"/>
        <include>
            <uri>model://ground_plane</uri>
        </include>

        <include>
            <uri>model://sun</uri>
        </include>
  </world>
</sdf>
```

```cpp title="world_plugin.cc"
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>

namespace gazebo
{
    class WorldPluginTutorial : public WorldPlugin
    {
    public:
        WorldPluginTutorial() : WorldPlugin()
        {
            gzmsg << "gazebo message \n";
            gzdbg << "gazebo debug \n";
            gzwarn << "gazebo warning \n";
            gzerr << "gazebo error \n";
        }

    public:
        void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
        {
        }
    };
    GZ_REGISTER_WORLD_PLUGIN(WorldPluginTutorial)
}
```

```c title="CMakeLists.txt
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(GAZEBO_PLUGIN_DEMO)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")


set(CMAKE_INSTALL_PREFIX "${PROJECT_SOURCE_DIR}")

add_library(hello_world SHARED src/world_plugin.cc)
target_link_libraries(hello_world ${GAZEBO_LIBRARIES})
install(TARGETS hello_world DESTINATION bin)
```

---

### Usage

!!! tip "GAZEBO_PLUGIN_PATH"
    Set `GAZEBO_PLUGIN_PATH` environment variable 

```
gazebo --verbose demo_world_plugin.world
```

![](images/gazebo_minimal_world_plugin_output.png)


---

## Resource
- [Gazebo plugin tutorial](https://sites.google.com/view/gazebo-plugin-tutorials/0-introduction?authuser=0)
- [Setting Velocity on Joints and Links](https://classic.gazebosim.org/tutorials?tut=set_velocity&cat=#SettingVelocityUsingPIDControllers)
    - [Setting Velocity on Joints and Links github](https://github.com/osrf/gazebo_tutorials/tree/master/set_velocity/examples/set_vel_plugin)
- [Plugins 101](https://classic.gazebosim.org/tutorials?tut=plugins_hello_world)
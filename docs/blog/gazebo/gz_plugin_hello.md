---
tags:
    - gazebo
    - gz
    - plugin
    - simulation
    - harmonic
---

# Plugin hello
Minimal model plugin

### Plugin code
```cpp title="MinimalModelPlugin.cc"
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/System.hh>
#include <gz/plugin/Register.hh>
 
using namespace gz;
using namespace sim;
using namespace systems;
 
// Inherit from System and 2 extra interfaces:
// ISystemConfigure and ISystemPostUpdate
class MyPlugin
      : public System,
        public ISystemConfigure,
        public ISystemPostUpdate
{
  // Implement Configure callback, provided by ISystemConfigure
  // and called once at startup.
  virtual void Configure(const Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         EntityComponentManager &_ecm,
                         EventManager &/*_eventMgr*/) override
  {
    // Read property from SDF
    auto linkName = _sdf->Get<std::string>("link_name");
 
    // Create model object to access convenient functions to get model link, joint and other model entities
    auto model = Model(_entity);
 
    // Get link entity
    this->linkEntity = model.LinkByName(_ecm, linkName);
  }
 
  // Implement PostUpdate callback, provided by ISystemPostUpdate
  // and called at every iteration, after physics is done
  virtual void PostUpdate(const UpdateInfo &/*_info*/,
                          const EntityComponentManager &_ecm) override
  {
    // Get link pose and print it
    std::cout << worldPose(this->linkEntity, _ecm) << std::endl;
  }
 
  // ID of link entity
  private: Entity linkEntity;
};
 
// Register plugin
GZ_ADD_PLUGIN(MyPlugin,
                    gz::sim::System,
                    MyPlugin::ISystemConfigure,
                    MyPlugin::ISystemPostUpdate)
 
// Add plugin alias so that we can refer to the plugin without the version
// namespace
GZ_ADD_PLUGIN_ALIAS(MyPlugin, "gz::sim::systems::MyPlugin")

```

#### ISystemConfigure
ISystemConfigure is an interface used in plugin development that allows you to execute custom code when a plugin is loaded and initialized within the simulation. its called once when the plugin is loaded and the simulation is being initialized.

the method `Configure` implement the interface

```cpp
void Configure(
    const gz::sim::Entity &entity,
    const std::shared_ptr<const sdf::Element> &sdf,
    gz::sim::EntityComponentManager &ecm,
    gz::sim::EventManager &eventMgr)
```

- entity: The entity to witch the plugin attach (model, world)
- sdf: the sdf part that declare the plugin
- ecm: Entity Component Manager allow access to entities and component in simulation
- eventMgr: provide access to simulation event , connect custom callback to sim events


#### ISystemPostUpdate
ISystemPostUpdate is an interface used in Gazebo system plugins to execute custom code after **each simulation update cycle**

```cpp
void PostUpdate(
    const gz::sim::UpdateInfo &info,
    const gz::sim::EntityComponentManager &ecm) override;
```

- info: sim information like simTime realTime play/pause
- ecm: use to get entities state like pose, velocities and sensor reading

---

### Entity Component System (ECS)

- **Entity**: a container for Component
- **Component**: data or property associate with entity like Pose, Velocity sensor data, no behavior

---

### CMakeLists.txt
```cmake
cmake_minimum_required(VERSION 3.8)
project(pov_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(gz-sim8 REQUIRED)
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})

add_library(MinimalModelPlugin SHARED src/MinimalModelPlugin.cc)
target_link_libraries(MinimalModelPlugin gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER})
```

```xml
<model name='vehicle_blue' canonical_link='chassis'>
         <plugin filename="MinimalModelPlugin" name="MyPlugin">
            <link_name>link</link_name>
        </plugin>

    <link name="link">
    </link>
    ...
</model>
```

### VSCode 

```json title="c_cpp_properties.json"
"includePath": [
          "${workspaceFolder}/**",
          "/usr/include",
          "/usr/local/include",
          "/usr/include/gz/sim8",
          "/usr/include/gz/math7",
          "/usr/include/gz/msgs10",
          "/usr/include/gz/common5",
          "/usr/include/gz/utils2",
          "/usr/include/gz/sdformat14",
          "/usr/include/gz/transport13",
          "/usr/include/gz/plugin2"
          
        ],
```
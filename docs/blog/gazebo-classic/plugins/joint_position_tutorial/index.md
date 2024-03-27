---
tags:
    - gazebo
    - classic
    - model
    - plugin
    - position
    - ros
---

Create gazebo model plugin in to control joint position using ROS2

- Basic plugin
- [Add ROS interface](ros_interface.md)
- [PID and Force](pid_and_set_force.md)

# project setup

```
├── .vscode
│   └── c_cpp_properties.json
├── bin
├── build
├── scripts
│   └── gazebo.sh
├── src
│   ├── CMakeLists.txt
│   └── pose_control.cpp
├── worlds
│   └── simple.world
├── model
│   └── position_demo
│       ├── CMakeLists.txt
│       └── pose_control.cpp
```


!!! tip ""
    ```
    "cppStandard": "c++17"
    ```

```json title="c_cpp_properties.json"
{
  "configurations": [
    {
      "browse": {
        "databaseFilename": "${default}",
        "limitSymbolsToIncludedHeaders": false
      },
      "includePath": [
        "/opt/ros/humble/include/**",
        "/usr/include/gazebo-11/**",
        "/usr/include/ignition/**",
      ],
      "name": "ROS",
      "intelliSenseMode": "gcc-x64",
      "compilerPath": "/usr/bin/gcc",
      "cStandard": "gnu11",
      "cppStandard": "c++17"
    }
  ],
  "version": 4
}
```

```bash title="gazebo.sh"
PROJECT=`pwd`
export GAZEBO_RESOURCE_PATH=${PROJECT}/worlds:${GAZEBO_RESOURCE_PATH}
export GAZEBO_PLUGIN_PATH=${PROJECT}/bin:${GAZEBO_PLUGIN_PATH}
export GAZEBO_MODEL_PATH=${PROJECT}/models:${GAZEBO_MODEL_PATH}
```

### Model

```xml title="model.conf"
<?xml version="1.0"?>

<model>
  <name>gimbal tester</name>
  <sdf version="1.7">model.sdf</sdf>

</model>

```

```xml title="model.sdf"
<?xml version='1.0'?>
<sdf version="1.7">
  <model name="gimbal_tester">
    <plugin name='set_joint_position_plugin' filename='libjoint_position_control_plugin.so'/>
    <frame name="base">
      <pose>0 0 0 0 0 0</pose>
    </frame>
    <link name="link">
      <pose relative_to="base">0 0 0.5 0 0 0</pose>
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
    <frame name="base_tip" attached_to="base">
      <pose relative_to="base">0 0 1 0 0 0</pose>
    </frame>
    <joint type="revolute" name="simple">
      <pose relative_to="base_tip">0 0 0 0 0 0</pose>
      <child>link2</child>
      <parent>link</parent>
      <axis>
        <xyz>0 0 1</xyz>
      </axis>
    </joint>

    <link name="link2">
      <pose relative_to="base_tip">0 0 0.25 0 0 0</pose>
      <collision name="collision">
        <geometry>
          <box>
            <size>0.5 0.5 0.5</size>
          </box>
        </geometry>
      </collision>
        <visual name="visual">
            <geometry>
              <box>
                <size>0.5 0.5 0.5</size>
              </box>
            </geometry>
            <material>
              <ambient>1 0 0 1</ambient>
              <diffuse>1 0 0 1</diffuse>
              <specular>1 0 0 1</specular>
          </material>
          </visual>
    </link>
  </model>
</sdf>
```
### Plugin
```c title="CMakeLists.txt"
add_library(joint_position_control_plugin
    SHARED 
    pose_control.cpp
)
target_link_libraries(joint_position_control_plugin ${GAZEBO_LIBRARIES})

install(TARGETS
joint_position_control_plugin
  DESTINATION ${PROJECT_SOURCE_DIR}/bin
)
```

```cpp title=""
// https://sites.google.com/view/gazebo-plugin-tutorials/9-gazebo-joint-move-plugin?authuser=0
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>


namespace gazebo{
class JointPositionControllerPlugin : public ModelPlugin{
public:
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
        gzmsg << "hello1 \n";
        this->joint = _model->GetJoint("simple");

        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&JointPositionControllerPlugin::OnUpdate, this));

    }

    void OnUpdate() {
        this->joint->SetPosition(0, 1.0, false);
        
    }
private:
    physics::ModelPtr model;
    physics::JointPtr joint;
    event::ConnectionPtr updateConnection;
};
GZ_REGISTER_MODEL_PLUGIN(JointPositionControllerPlugin)
}
```

---

## Gazebo

![](images/joint_position_basic.png)
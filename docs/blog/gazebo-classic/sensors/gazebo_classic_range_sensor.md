---
tags:
    - gazebo
    - classic
    - sensors
    - range
---


```xml title="ray"
<sensor name="mtl" type="ray">
    <visualize>true</visualize>
    <update_rate>10</update_rate>
    <ray>
        <scan>
            <horizontal>
            <samples>10</samples>
            <resolution>1.00000</resolution>
            <min_angle>-0.174533</min_angle>
            <max_angle>0.174533</max_angle>
            </horizontal>
        </scan>
        <range>
            <min>0.5</min>
            <max>5</max>
            <resolution>0.1</resolution>
        </range>
        </ray>
</sensor>
```

## plugin 
Read sensor data using plugin

!!! note "Sensor plugin"
    Place the plugin in sensor scope

    ```xml
    <sensor name="range_sensor" type="ray">
        ....
        <plugin name="range_sensor_plugin" filename="libRangeSensorPlugin.so"/>
    </sensor>
    ```

```cpp title="RangeSensorPlugin"
// code generate by phind
#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>

namespace gazebo
{
 class RangeSensorPlugin : public SensorPlugin
 {
    public: void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
    {
      // Initialize the sensor
      this->parentSensor = std::dynamic_pointer_cast<sensors::RaySensor>(_sensor);
      if (!this->parentSensor)
      {
        gzerr << "RangeSensorPlugin requires a RaySensor.\n";
        return;
      }

      // Connect to the sensor's update event
      this->updateConnection = this->parentSensor->ConnectUpdated(
          std::bind(&RangeSensorPlugin::OnUpdate, this));
    }

    public: void OnUpdate()
    {
      // Get the range data
      auto rangeData = this->parentSensor->Range(0);

      // Print the range data
      gzmsg << "Range: " << rangeData << std::endl;
    }

    private: sensors::RaySensorPtr parentSensor;
    private: event::ConnectionPtr updateConnection;
 };

 GZ_REGISTER_SENSOR_PLUGIN(RangeSensorPlugin)
}

```

```c
find_package(gazebo REQUIRED)


include_directories(
    ${GAZEBO_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/include
)
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(RangeSensorPlugin SHARED RangeSensorPlugin.cpp)
target_link_libraries(RangeSensorPlugin ${GAZEBO_LIBRARIES})

install(TARGETS
RangeSensorPlugin
  DESTINATION ${PROJECT_SOURCE_DIR}/bin
)

```
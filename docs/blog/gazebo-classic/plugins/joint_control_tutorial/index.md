---
tags:
    - gazebo classic
    - plugin
    - joint
    - velocity
    - sensor
    - imu
---

Control Joint from gazebo plugin using
- Position
- Force

Add IMU sensor to control joint relative to world coordinate


## Get joint
- Read joint name from SDF file
- Get joint from model

```cpp
std::string jointName = "tilt_joint";
  if (_sdf->HasElement("joint"))
  {
    jointName = _sdf->Get<std::string>("joint");
  }
  auto tiltJoint = _model->GetJoint(jointName);
```

## Add Sensor

- Add sensor include
- Register sensor in plug init or load
- Read sensor data

### Demo: IMU sensor

```c++
#include <gazebo/sensors/sensors.hh>
```

```c++ title="Load"
std::string imuSensorName = "roll_imu";
  if (_sdf->HasElement("imu_sensor"))
  {
    imuSensorName = _sdf->Get<std::string>("imu_sensor");
  }

  this->imuSensor = std::static_pointer_cast<sensors::ImuSensor>(
      sensors::SensorManager::Instance()->GetSensor(_model->SensorScopedName(imuSensorName)[0]));
```

```cpp title="read sensor data"
auto roll = this->imuSensor->Orientation().Euler().X(); 
```

---

## Reference
- [Setting Velocity on Joints and Links](https://classic.gazebosim.org/tutorials?tut=set_velocity&cat=)
- [Gazebo plugin tutorial](https://sites.google.com/view/gazebo-plugin-tutorials/0-introduction?authuser=0)

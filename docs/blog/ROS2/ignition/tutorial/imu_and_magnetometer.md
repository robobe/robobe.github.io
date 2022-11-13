---
title: IMU
tags:
    - imu
    - ros2
    - ignition
---

# LAB
- Add IMU sensor to sdf
- Bridge to ROS2
- Show in RVIZ


## sdf
- Add sensor sdf under `link`
- Add imu plugin 

Add plugin tag under `world` tag
```xml
<plugin filename="libignition-gazebo-imu-system.so"
        name="ignition::gazebo::systems::Imu">
</plugin>
```

Add sensor to `imu_link` tag
```xml
<sensor name="imu_sensor" type="imu">
    <always_on>1</always_on>
    <update_rate>1</update_rate>
    <visualize>true</visualize>
    <topic>imu</topic>
</sensor>
```

Check this link for [more info](https://github.com/gazebosim/docs/blob/master/dome/sensors.md)
---

## tf
- Add static tf between sensor frame to link frame

!!! note "static tf"
    Sensor has fix frame_id, set from it's location
    We need to set static tf form link frame to sensor frame_id
     
```python
imu_tf = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name="imu2imu_link",
    arguments = ["0", "0", "0", "0", "0", "0", "imu_link", "basic_mobile_bot/imu_link/imu"]
)
```

![](images/imu_and_magnetometer_tf.png)

---

## bridge
[bridge source code ](files/imu_and_magnometer/ign_bridge.launch.py)

- Add imu msg bridge
  - Set QoS to best_effort
- Add tf static transform from `imu_link` to sensor frame_id `basic_mobile_bot/imu_link/imu`
  



```python title="bridge" linenums="1" hl_lines="8"
imu_bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=['/imu@sensor_msgs/msg/Imu@gz.msgs.IMU'],
    output='screen',
    parameters=[{
        'use_sim_time': use_sim_time,
        'qos_overrides./imu.publisher.reliability': 'best_effort'
    }]
)

```

```bash title="before qos convert"
ros2 topic info /imu --verbose
Type: sensor_msgs/msg/Imu

Publisher count: 1

Node name: ros_gz_bridge
Node namespace: /
Topic type: sensor_msgs/msg/Imu
Endpoint type: PUBLISHER
GID: 01.0f.64.c6.90.63.0b.31.01.00.00.00.00.00.14.03.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: RELIABLE
  History (Depth): UNKNOWN
  Durability: VOLATILE
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite
```

```bash title="imu message after qos convert"
ros2 topic info /imu --verbose
#
Type: sensor_msgs/msg/Imu

Publisher count: 1

Node name: ros_gz_bridge
Node namespace: /
Topic type: sensor_msgs/msg/Imu
Endpoint type: PUBLISHER
GID: 01.0f.64.c6.fb.16.9f.99.01.00.00.00.00.00.14.03.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: BEST_EFFORT
  History (Depth): UNKNOWN
  Durability: VOLATILE
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite

```

---

# usage

```bash title="terminal1"
ros2 launch ign_tutorial ign.launch.py with_bridge:=true
```

```bash title="terminal2"
ros2 run rqt_robot_steering rqt_robot_steering
```

```bash title="terminal3"
rviz
```

![](images/imu_and_magnetometer.gif)

---

# Reference
- [ros_gz_sim_demos](https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_sim_demos/launch/imu.launch.py)
- [IMU tools for ROS](https://github.com/CCNYRoboticsLab/imu_tools/tree/humble)
- [ignition sensors](https://github.com/gazebosim/docs/blob/master/dome/sensors.md)
- [rviz imu marker](https://medium.com/@hitlx916/visualize-imu-msg-from-scratch-2d-ros-rviz-b7869a804e36)
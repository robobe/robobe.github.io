---
tags:
    - gazebo
    - ros2
    - sensors
    - ray
    - gazebo_ros_ray_sensor
    - lidar
    - ultrsonic
    - range
---
# ROS2 gazebo (classic) LIDAR sensor
ROS2 using new plug `gazebo_ros_ray_sensor` to simulate `PointCloud`, `LaserScan` and `Range` output message control by `<output_type>` tag


- **sensor_msgs/PointCloud2**: 3D cloud of points (default)
- **sensor_msgs/PointCloud**: 3D cloud of points
- **sensor_msgs/LaserScan**: 2D scan, uses center vertical ray if there are multiple
- **sensor_msgs/Range**: Single distance value, minimum of all ray ranges of parent sensor



## Demo
### SDF

```xml
<?xml version="1.0"?>
<sdf version="1.5">
    <model name="lidar_demo">
        <pose>0 0 0.5 0 0 0</pose>
        <static>true</static>
        <link name="link">
            <collision name="collision">
                <geometry>
                    <box>
                        <size>0.1 0.1 0.1</size>
                    </box>
                </geometry>
            </collision>
            <visual name="visual">
                <geometry>
                    <box>
                        <size>0.1 0.1 0.1</size>
                    </box>
                </geometry>
            </visual>
            <sensor name="lidar" type="ray">
                <always_on>true</always_on>
                <visualize>true</visualize>
                <update_rate>5</update_rate>
                <ray>
                  <scan>
                    <horizontal>
                      <samples>180</samples>
                      <resolution>1.00000</resolution>
                      <min_angle>-1.57</min_angle>
                      <max_angle>1.57</max_angle>
                    </horizontal>
                  </scan>
                  <range>
                    <min>0.5</min>
                    <max>3.5</max>
                    <resolution>0.1</resolution>
                  </range>
                </ray>
                <plugin name="scan" filename="libgazebo_ros_ray_sensor.so">
                  <ros>
                    <remapping>~/out:=scan</remapping>
                  </ros>
                  <output_type>sensor_msgs/LaserScan</output_type>
                  <frame_name>link</frame_name>
                </plugin>
              </sensor>
        </link>
    </model>
</sdf>
```     

---

### Run
#### sensor_msgs/Range
```xml
<plugin name="ultrasonic_sensor" filename="libgazebo_ros_ray_sensor.so">
    <ros>
        <remapping>~/out:=range</remapping>
    </ros>
    <output_type>sensor_msgs/Range</output_type>
    <frame_name>link</frame_name>
</plugin>
```

```bash
ros2 topic info /range
#
Type: sensor_msgs/msg/Range
Publisher count: 1
```

```bash
header:
  stamp:
    sec: 2978
    nanosec: 715000000
  frame_id: link
radiation_type: 1
field_of_view: 0.23999999463558197
min_range: 0.5
max_range: 2.5
range: 2.013324022293091
```

!!! warning ""
    This type of message raise exception when reading is out of range

    ```bash
    ros2 topic echo /range 
    #
    Unable to convert call argument to Python object (compile in debug mode for details)
    ```

    I Open an [issue](https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1474) for this exception
     
---

### sensor_msgs/LaserScan

```xml
<plugin name="ultrasonic_sensor" filename="libgazebo_ros_ray_sensor.so">
    <output_type>sensor_msgs/LaserScan</output_type>
    <frame_name>link</frame_name>
</plugin>
```

```bash
ros2 topic info /ultrasonic_sensor/out
# 
Type: sensor_msgs/msg/LaserScan
```

```bash
header:
  stamp:
    sec: 109
    nanosec: 415000000
  frame_id: link
angle_min: -0.11999999731779099
angle_max: 0.11999999731779099
angle_increment: 0.05999999865889549
time_increment: 0.0
scan_time: 0.0
range_min: 0.5
range_max: 2.5
ranges:
- .inf
- 1.8191075325012207
- 1.7666829824447632
- 1.8185886144638062
- .inf
intensities:
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0

```

!!! note ".inf"
    When a beam is out of range (min/max) it's return `.inf`
     
---

## Reference
- [ROS 2 Migration: Ray sensors](https://github.com/ros-simulation/gazebo_ros_pkgs/wiki/ROS-2-Migration:-Ray-sensors#gazebo_ros_range)

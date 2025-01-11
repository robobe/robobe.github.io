---
tags:
    - gazebo
    - gps
    - gz
    - sensor
    - simulation
    - harmonic
---

# GPS/NavSat Sensor

[navsat world](https://github.com/gazebosim/gz-sim/blob/gz-sim9/test/worlds/navsat.sdf)

```xml
 <plugin
    filename="gz-sim-navsat-system"
    name="gz::sim::systems::NavSat">
</plugin>
```

```xml
<sensor name="navsat_sensor" type="navsat">
    <always_on>1</always_on>
    <update_rate>30</update_rate>
    <topic>gps</topic>
    <gz_frame_id>gps_frame</gz_frame_id>
</sensor>
```

```xml
<spherical_coordinates>
    <surface_model>EARTH_WGS84</surface_model>
    <world_frame_orientation>ENU</world_frame_orientation>
    <latitude_deg>-22.9</latitude_deg>
    <longitude_deg>-43.2</longitude_deg>
    <elevation>0</elevation>
    <heading_deg>0</heading_deg>
</spherical_coordinates>
```

---

### Bridge

```
ros2 run ros_gz_bridge parameter_bridge  /gps@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat
```


---
tags:
    - gazebo
    - garden
    - lidar
    - urdf
---

# Add lidar sensor to urdf file

```xml
<gazebo reference="lidar_link">
    <sensor name="lidar" type="gpu_lidar">
        <topic>scan</topic>
        <always_on>true</always_on>
        <visualize>true</visualize>
        <update_rate>5</update_rate>
        <gz_frame_id>lidar_link</gz_frame_id>
        <ray>
            <scan>
                <horizontal>
                    <samples>360</samples>
                    <resolution>1.000000</resolution>
                    <min_angle>0.000000</min_angle>
                    <max_angle>6.280000</max_angle>
                </horizontal>
            </scan>
            <range>
                <min>0.120000</min>
                <max>3.5</max>
                <resolution>0.015000</resolution>
            </range>
            <noise>
                <type>gaussian</type>
                <mean>0.0</mean>
                <stddev>0.01</stddev>
            </noise>
        </ray>
    </sensor>
</gazebo>
```




![](images/robot_with_lidar.png)
---
tags:
    - gazebo
    - imu
    - gz
    - sensor
    - simulation
    - harmonic
---

# IMU Sensor
- Add sensor to **link**
- Add Sensor IMU plugin to **world**
  


       
```xml
<sensor name="imu_sensor" type="imu">
    <always_on>1</always_on>
    <update_rate>1</update_rate>
    <visualize>true</visualize>
    <topic>imu</topic>
</sensor>
```

```xml
<plugin
    filename="gz-sim-imu-system"
    name="gz::sim::systems::Imu">
</plugin>

```

!!! note "gz::sim::system:Sensors"
    The gz::sim::systems::Sensors class is a system plugin that allows interaction with sensors within a simulation environment. It provides functionality for:

     

### More control
#### Control topic and reference frame

- topic
- gz_frame_id

```
<sensor name="imu_sensor" type="imu">
    <pose degrees="true">0 0 0 180 0 0</pose>
    <always_on>1</always_on>
    <update_rate>1000</update_rate>
    <topic>imu</topic>
    <gz_frame_id>vehicle_blue/chassis</gz_frame_id>
</sensor>
```

#### Add Noise
- angular_velocity
- linear_acceleration

```xml
<sensor name="imu_sensor" type="imu">
    <pose>0 0 0.1 0 0 0</pose>
    <update_rate>100.0</update_rate> <!-- Update rate in Hz -->
          
    <imu>
        <!-- Noise settings for angular velocity (X, Y, Z axes) -->
        <angular_velocity>
            <x>
              <noise type="gaussian">
                <mean>0</mean>
                <stddev>0.009</stddev>
                <bias_mean>0.00075</bias_mean>
                <bias_stddev>0.005</bias_stddev>
                <dynamic_bias_stddev>0.00002</dynamic_bias_stddev>
                <dynamic_bias_correlation_time>400.0</dynamic_bias_correlation_time>
                <precision>0.00025</precision>
              </noise>
            </x>
        </angular_velocity>
        <linear_acceleration>
        </linear_acceleration>
    </imu>
</sensor>
```

#### orientation_reference_frame
- NED
- ENU
- Custom

[more](https://github.com/gazebosim/gz-sim/blob/gz-sim9/test/worlds/imu_heading_deg.sdf)

```xml
<sensor name='imu_sensor_NED' type='imu'>
    <topic>imu_test_NED</topic>
    <update_rate>1</update_rate>
    <imu>
    <orientation_reference_frame>
        <localization>NED</localization>
    </orientation_reference_frame>
    </imu>
    <always_on>1</always_on>
    <visualize>true</visualize>
</sensor>
```

---

## Demo

- [imu sensor](https://github.com/gazebosim/gz-sim/blob/gz-sim9/test/worlds/imu.sdf)
- [imu sensor with orientation control (ned,enu,..)](https://github.com/gazebosim/gz-sim/blob/gz-sim9/test/worlds/imu_heading_deg.sdf)
```xml
<?xml version="1.0"?>
<sdf version="1.7">
    <world name="default">
        <scene>
            <ambient>0.5 0.5 0.5 1</ambient>
        </scene>

        <physics name="1ms" type="ignored">
            <max_step_size>0.001</max_step_size>
            <real_time_factor>1.0</real_time_factor>
        </physics>
        <plugin
            filename="gz-sim-physics-system"
            name="gz::sim::systems::Physics">
        </plugin>

        <plugin
            filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster">
        </plugin>
        <plugin
            filename="gz-sim-imu-system"
            name="gz::sim::systems::Imu">
        </plugin>
        <plugin
            filename="gz-sim-sensors-system"
            name="gz::sim::systems::Sensors">
            <render_engine>ogre2</render_engine>
        </plugin>


        <include>
      <uri>model://ground_plane</uri>
    </include>

        <!-- A simple robot with an IMU sensor -->
        <model name="robot_with_imu">
            <link name="link">
                <inertial>
                    <mass>2</mass>
                    <inertia>
                        <ixx>0.041666667</ixx>
                        <ixy>0</ixy>
                        <ixz>0</ixz>
                        <iyy>0.056666667</iyy>
                        <iyz>0</iyz>
                        <izz>0.068333333</izz>
                    </inertia>
                </inertial>
                <collision name="collision">
                    <geometry>
                        <box>
                            <size>0.5 0.4 0.3</size>
                        </box>
                    </geometry>
                    <surface>
                        <friction>
                            <ode>
                                <mu>1.0</mu>
                                <mu2>1.0</mu2>
                            </ode>
                        </friction>
                        <contact>
                            <ode>
                                <kp>10000000.0</kp>
                                <kd>1.0</kd>
                                <min_depth>0.001</min_depth>
                                <max_vel>0.1</max_vel>
                            </ode>
                        </contact>
                    </surface>
                </collision>
                <visual name="visual">
                    <pose>0 0 -0.15 0 0 0</pose>
                    <geometry>
                        <box>
                            <size>0.5 0.4 0.3</size>
                        </box>
                    </geometry>
                </visual>
                <!-- IMU  -->
                <sensor name="imu_sensor" type="imu">
                    <always_on>1</always_on>
                    <update_rate>1</update_rate>
                    <visualize>true</visualize>
                    <topic>imu</topic>
                </sensor>
            </link>
        </model>
    </world>
</sdf>
```
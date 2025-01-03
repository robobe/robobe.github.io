---
tags:
    - gazebo
    - imu
    - gz
    - simulation
---

# IMU Sensor
- Add sensor to link
- Add Sensor and imu plugin
  
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
<plugin
    filename="gz-sim-sensors-system"
    name="gz::sim::systems::Sensors">
    <render_engine>ogre2</render_engine>
</plugin>
```

!!! note "gz::sim::system:Sensors"
    The gz::sim::systems::Sensors class is a system plugin that allows interaction with sensors within a simulation environment. It provides functionality for:

    Accessing sensor data.
    Updating sensor readings.
    Handling sensor events.
    Enabling and disabling sensors.
     

### More control
- Add Noise

```xml
<sensor name="imu_sensor" type="imu">
    <pose>0 0 0.1 0 0 0</pose>
    <update_rate>100.0</update_rate> <!-- Update rate in Hz -->
          
    <imu>
        <!-- Noise settings for angular velocity (X, Y, Z axes) -->
        <angular_velocity>
            <noise>
            <type>gaussian</type>      <!-- Gaussian noise model -->
            <mean>0.0</mean>           <!-- Mean of the noise -->
            <stddev>0.01</stddev>      <!-- Standard deviation (noise level) -->
            </noise>
        </angular_velocity>
    </imu>
</sensor>
```

---

## Demo

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
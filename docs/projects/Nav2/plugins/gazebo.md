---
title: Part2 - Nav2 basic two wheel robot
description: Gazebo simulation for two wheel robot plugins
date: "2022-05-08"
banner: ../ros2.png
tags:
    - ros2
    - Nav2
    - gazebo
    - diff_drive
    - camera
    - imu
    - lidar
---

# Gazebo
```bash title=""install"

```
## launch
## spawn

---

# diff_drive
## plugin
```xml
 <model name='vehicle'>
      ...

      <joint name='left_wheel_joint' type='revolute'>
        ...
      </joint>

      <joint name='right_wheel_joint' type='revolute'>
        ...
      </joint>

      <!-- Use gazebo_ros_joint_state_publisher instead of publishWheelJointState -->
      <plugin name="joint_states" filename="libgazebo_ros_joint_state_publisher.so">
        <joint_name>right_wheel_joint</joint_name>
        <joint_name>left_wheel_joint</joint_name>
      </plugin>


      <plugin name='diff_drive' filename='libgazebo_ros_diff_drive.so'>
        <ros>
          <!-- Set namespace -->
          <namespace>/demo</namespace>

          <!-- Remap default topics -->
          <argument>cmd_vel:=cmd_demo</argument>
          <argument>odom:=odom_demo</argument>
        </ros>

        <!-- Replace camelCase elements with camel_case ones -->
        <update_rate>500</update_rate>
        <left_joint>left_wheel_joint</left_joint>
        <right_joint>right_wheel_joint</right_joint>
        <wheel_separation>1.25</wheel_separation>
        <wheel_diameter>0.3</wheel_diameter>
        <odometry_frame>odom</odometry_frame>

        <!-- wheelTorque and wheelAcceleration now have max_ prefix -->
        <max_wheel_torque>20</max_wheel_torque>
        <max_acceleration>1.0</max_acceleration>

      </plugin>

    </model>
```

---

## joint_state

---

# Reference
- [Installing gazebo_ros_pkgs](https://classic.gazebosim.org/tutorials?tut=ros2_installing&cat=connect_ros)
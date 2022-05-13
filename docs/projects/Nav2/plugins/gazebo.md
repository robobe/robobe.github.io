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
sudo apt install ros-foxy-gazebo-ros-pkgs
```
## launch
## spawn

---

# diff_drive
## plugin
```xml
 <model name='vehicle'>
      ...

      <joint name='drivewhl_l_joint' type='revolute'>
        ...
      </joint>

      <joint name='drivewhl_r_joint' type='revolute'>
        ...
      </joint>

      <!-- Use gazebo_ros_joint_state_publisher instead of publishWheelJointState -->
      <plugin name="joint_states" filename="libgazebo_ros_joint_state_publisher.so">
        <joint_name>drivewhl_r_joint</joint_name>
        <joint_name>drivewhl_l_joint</joint_name>
      </plugin>


      <plugin name='diff_drive' filename='libgazebo_ros_diff_drive.so'>
        <ros>
        <namespace>/skbot</namespace>
      </ros>

      <!-- wheels -->
      <left_joint>drivewhl_l_joint</left_joint>
      <right_joint>drivewhl_r_joint</right_joint>

      <!-- kinematics -->
      <wheel_separation>${base_width+wheel_width}</wheel_separation>
      <wheel_diameter>${2*wheel_radius}</wheel_diameter>

      <!-- limits -->
      <max_wheel_torque>10</max_wheel_torque>
      <max_wheel_acceleration>1.0</max_wheel_acceleration>

      <!-- output -->
      <publish_odom>true</publish_odom>
      <publish_odom_tf>true</publish_odom_tf>

      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base_link</robot_base_frame>
    </plugin>

    </model>
```

---

## joint_state

The `joint state publisher` plugin (`libgazebo_ros_joint_state_publisher.so`) will publish the angles of the drive wheel joints, which are continuously changing once the robot starts moving.

!!! Note
    In a real robotics project, to calculate the odometry system using `IMU` and `wheel encoder`
---

# Reference
- [Installing gazebo_ros_pkgs](https://classic.gazebosim.org/tutorials?tut=ros2_installing&cat=connect_ros)
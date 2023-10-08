---
title: Odometry
tags:
    - ros2
---

odometry using data from sensors to estimate the change in a robot’s position, orientation, and velocity over time relative to some point (e.g. x=0, y=0, z=0). Odometry usually used sensors like
- Wheel Encoder
- IMU
- LIDAR

ROS used `odom` frame to mark the point in the world where the robot first start to moving.


---
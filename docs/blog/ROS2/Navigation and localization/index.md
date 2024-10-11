---
tags:
    - navigation
    - localization
    - odometry
    - ros2
---


## Odometry
Process of using motion sensors to detect change in position
- encoders
- visual
- Distance
- 

### Map frame
- Global frame, fixed reference frame 
- Its serve as the root of the coordinate system for navigation

### Odom frame
- Local frame, fixed frame relative to the robot starting position
- The odom frame drift over time due to accumulated error in odometry calculation

### Base link frame
- The base_link attach to fixed position on the robot
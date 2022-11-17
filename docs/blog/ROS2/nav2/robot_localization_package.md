---
title: Robot Localization Package
tags:
    - nav2
    - ros2
---

robot_localization: Provides nonlinear state estimation through sensor fusion of an arbitrary number of sensors.

# LAB
Use robot_localization to fuse odometry and imu reading using `ekf_node`


## install

```
sudo apt install ros-humble-robot-localization
```

## usage

```python
robot_localization_file_path = os.path.join(pkg, 'config/ekf.yaml') 

# Start robot localization using an Extended Kalman filter
  start_robot_localization_cmd = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    output='screen',
    parameters=[robot_localization_file_path, 
    {'use_sim_time': use_sim_time}])
```
---

# Reference
- [robot_localization wiki¶](http://docs.ros.org/en/noetic/api/robot_localization/html/index.html)
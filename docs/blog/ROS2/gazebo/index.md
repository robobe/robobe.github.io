---
title: ros2 gazebo integration
tags:
    - ros2
    - gazebo-classic
---

```bash
sudo apt install ros-humble-gazebo-ros-pkgs
```

## test integration
```bash
gazebo --verbose /opt/ros/humble/share/gazebo_plugins/worlds/gazebo_ros_diff_drive_demo.world
```

```
ros2 topic pub /demo/cmd_demo geometry_msgs/Twist '{linear: {x: 1.0}}' -1
```

---

# Reference
- [Installing gazebo_ros_pkgs (ROS 2)](http://classic.gazebosim.org/tutorials?tut=ros2_installing&cat=connect_ros)
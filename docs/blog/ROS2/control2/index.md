---
title: ROS2 Control_2
tags:
    - ros2
    - control2
---

```
sudo apt install ros-humble-ros2-control
sudo apt install ros-humble-ros2-controllers
sudo apt install ros-humble-gazebo-ros2-control
```


---

# Demo usage

```bash
 ros2 control list_hardware_interfaces 
 ros2 run controller_manager spawner diff_cont
 ros2 run controller_manager spawner joint_broad

```

```bash terminal="teleop"
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped
```

---

# Resource
- [#12 - ros2_control Concept & Simulation](https://articulatedrobotics.xyz/mobile-robot-12-ros2-control/)
- [ Solving the problem EVERY robot has (with ros2_control) ](https://youtu.be/4QKsDf1c4hc)
---
title: Ignition ROS2
tags:
    - ros_ign
---

## ros ign bridge

ros_ign contains packages that provide integration between ROS and Ignition:

- ros_ign: Metapackage that provides all other software packages;
- ros_ign_image: Use image_transport to transfer the image from Ignition to the one-way transmission bridge of ROS;
- ros_ign_bridge: Two-way transmission bridge between Ignition and ROS;
- ros_ign_gazebo: It is convenient to use the startup files and executable files of Ignition Gazebo and ROS;
- ros_ign_gazebo_demos: Demos using ROS-Ignition integration;
- ros_ign_point_cloud: A plug-in used to simulate publishing point clouds to ROS from Ignition Gazebo

### install
```
sudo apt install ros-humble-ros-gz
```

---

## launch
```bash
ros2 launch ros_ign_gazebo ign_gazebo.launch.py gz_args:="-r camera_sensor.sdf"
```



---

# Reference
- [ROS + Gazebo Sim demos](https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_sim_demos)
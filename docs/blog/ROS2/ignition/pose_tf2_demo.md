---
title: GZ Pose to ROS TF
tags:
    - pose
    - tf2
---

# Lab
- Send `tf` data from simulation using gz_bridge

---

# Setup
- Ignition fortress
- ROS2 Humble
- gz_ros_bridge

# Project

```
├── CMakeLists.txt
├── config
│   └── rviz.rviz
├── launch
│   └── box.launch.py
├── models
│   └── vehicle
│       ├── model.config
│       └── model.sdf
├── package.xml
└── worlds
    └── box.sdf
```

# PosePublisher
[PosePublisher](https://gazebosim.org/api/sim/6/classignition_1_1gazebo_1_1systems_1_1PosePublisher.html)

Pose publisher system. Attach to an entity to publish the transform of its child entities in the form of `ignition::msgs::Pose` messages or r a single `ignition::msgs::Pose_V` message if `use_pose_vector_msg` is true
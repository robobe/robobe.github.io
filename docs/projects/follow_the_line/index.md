---
title: Follow the line
tags:
    - ros2
    - projects
---

## Project
- 


```
├── basic_mobile_robot
│   ├── follow_line.py
│   ├── follow.py
│   ├── __init__.py
│   └── viewer.py
├── CMakeLists.txt
├── config
│   └── rviz.rviz
├── description
│   ├── camera.xacro
│   ├── depth_camera.xacro
│   ├── face.xacro
│   ├── gazebo_control.xacro
│   ├── inertial_macros.xacro
│   ├── lidar.xacro
│   ├── robot_core.xacro
│   ├── robot.urdf.xacro
│   └── ros2_control.xacro
├── launch
│   ├── articubot_one
│   │   ├── articubot_one.launch.py
│   │   └── rsp.launch.py
│   └── follow
│       └── follow.launch.py
├── models
│   └── ground_course
│       ├── materials
│       │   ├── scripts
│       │   │   └── course.material
│       │   └── textures
│       │       └── course.png
│       ├── model.config
│       └── model.sdf
├── package.xml
├── src
└── worlds
    └── course.world
```

## Demo

```bash
ros2 launch basic_mobile_robot follow.launch.py
```

![](images/follow_gazebo.png)
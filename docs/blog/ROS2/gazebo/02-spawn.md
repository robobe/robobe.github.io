---
title: Part2 - spawn
description: launch gazebo and spawn entity
tags:
    - gazebo-classic
    - tutorial
    - spawn
---

## project
[my ros2_gazebo_tutorial](https://github.com/robobe/ros2_gazebo_tutorial)

```
├── CMakeLists.txt
├── launch
│   └── spawn.launch.py
├── package.xml
├── worlds
│   └── empty.world
└── models
    └── simple
        ├── model.config
        └── model.sdf.xacro

```

## launch

```python title="basic_gazebo.launch.py" linenums="1" hl_lines="46 50 61"
--8<-- "/home/user/ros2_ws/src/ros2_gazebo_tutorial/launch/spawn.launch.py"
```
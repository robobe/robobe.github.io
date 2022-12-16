---
title: Turtlebot3 
tags:
    - turtlebot
    - tutorial
    - ros2
---

```
sudo apt install ros-humble-turtlebot3-gazebo
sudo apt install ros-humble-turtlebot3-teleop
```

```terminal1
export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:`ros2 pkg \
prefix turtlebot3_gazebo \
`/share/turtlebot3_gazebo/models/
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

```terminal2
export TURTLEBOT3_MODEL=burger
ros2 run turtlebot3_teleop teleop_keyboard
```

---

# Reference
- [TurtleBot in ROS 2](https://ros2-industrial-workshop.readthedocs.io/en/latest/_source/navigation/ROS2-Turtlebot.html)

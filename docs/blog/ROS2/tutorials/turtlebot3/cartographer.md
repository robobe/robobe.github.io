---
title: Turtlebot Cartographer
tags:
    - turtlebot
    - tutorial
    - ros2
    - nav
---

How to use Cartographer for mapping and localization

```bash
sudo apt install ros-humble-cartographer
sudo apt install ros-humble-cartographer-ros
sudo apt install ros-humble-turtlebot3-cartographer
```

```bash
export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH=`ros2 pkg \
prefix turtlebot3_gazebo`/share/turtlebot3_gazebo/models/
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

![](images/tutrtlebot_world.png)

![](images/turtlebot_rviz_map.png)


---

# Reference
- [Cartographer](https://ros2-industrial-workshop.readthedocs.io/en/latest/_source/navigation/ROS2-Cartographer.html)
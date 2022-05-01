---
title: Turtlebot3 simulation
description: Turtlebot3 simulation
date: "2022-04-30"
banner: ../../images/turtlebot3_logo.png
tags:
    - ros2
    - Turtlebot3 
    - simulation
---

# Installation
```bash
cd ~/turtlebot3_ws/src/
# turtlebot3
git clone -b foxy-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
# turtlebot3 msgs
git clone -b foxy-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
# simulation
git clone -b foxy-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
# build
cd ~/turtlebot3_ws && colcon build --symlink-install
```

# Run
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo empty_world.launch.py
```
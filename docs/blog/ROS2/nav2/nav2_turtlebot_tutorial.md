---
tags:
    - nav2
    - ros2
    - turtlebot
---

# Turtlebot3 nav2 tutorial

## demo

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

```bash
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
```

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

create map

![](images/turtlebot_map_done.png)

save map

```bash
ros2 run nav2_map_server map_saver_cli -f tmp/maps/my_map
```

## Localization

```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=/home/user/tmp/maps/my_map.yaml
```


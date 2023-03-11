---
title: NAV2 
tags:
    - nav
    - slam
---

## Coordinate frames
- base_link
- odom: world origin
- map
- base_footprint: like base_link but keep in 2D

!!! note Odometry
    Measure robot velocity  
    Velocity integrated to estimate position (dead reckoning)
     

![](images/map_odom_base_link.drawio.png)

### TF's
odom: Odometry Origin
map: World origin

### Topics
/odom: nav_msgs/msg/Odometry
/map: nav_msgs/msg/OccupancyGrid


---

## install 

```bash
sudo apt install ros-humble-slam-toolbox
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
```

---

## slam-toolbox

Online: Working in a live data (not recorded)
Asynchronous: Always process the most recent scan , avoid lagging (skip scan if lagging)

```
ros2 launch slam_toolbox online_async_launch.py params_file:=./src/basic_mobile_robot/config/slam_async.yaml use_sim_time:=true
```


![](images/rviz_slam_panel.png)

- Save map: Old format, create pgm and yaml files
- Serialize map: New format , create posegraph and data file


!!! tip "slam_toolbox high cpu"
    change param `do_loop_closing` to false 

     

## AMCL
### map server
```bash
ros2 run \
nav2_map_server \
map_server --ros-args \
-p yaml_filename:=my_map.yaml 
-p use_sim_time:=true

# Activate
ros2 run nav2_util \
lifecycle_bringup map_server

# test
ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap "{map_url: /home/user/nav2_ws/new_map2.yaml }"
```

### amcl
```bash
ros2 run nav2_amcl amcl --ros-args -p use_sim_time:=true

# Activate
ros2 run nav2_util lifecycle_bringup amcl
```

```
[WARN] [1676219701.288565313] [amcl]: AMCL cannot publish a pose or update the transform. Please set the initial pose...

```


### launch file

#### map server
```python
Node(
    package='nav2_map_server',
    executable='map_server',
    name='map_server',
    output='screen',
    parameters=[
        {'use_sim_time': True},
        {'yaml_filename':map_file}]
    )
```

#### lifecycle_manager
```python
Node(
    package='nav2_lifecycle_manager',
    executable='lifecycle_manager',
    name='lifecycle_manager',
    output='screen',
    parameters=[
        {'autostart': True},
        {'node_names': [
            'map_server',
            'amcl']
        }
    ]
)
```

![](images/amcl.drawio.png)

---

# Reference

- [ Easy SLAM with ROS using slam_toolbox](https://www.youtube.com/watch?v=ZaiA3hWaRzE&list=PLunhqkrRNRhYAffV8JDiFOatQXuU-NnxT&index=17)
- [articubot_one source code](https://github.com/joshnewans/articubot_one/)
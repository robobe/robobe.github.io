


# slam_toolbox
```
sudo apt install ros-humble-slam-toolbox
```


# mapping

```
ros2 launch slam_toolbox online_async_launch.py
```

**Online**: Working on a live data stream rather then recorded log
Asynchronous: Always process the most recent scan to avoid lagging, event if that mean skipping scan

## config
```
cp /opt/ros/humble/share/slam_toolbox/config/mapper_params_online_async.yaml src/nav_bringup/config 
```
## save map
```
ros2 run nav2_map_server map_saver_cli -f my_map
```

---

# Reference
- [Easy SLAM with ROS using slam_toolbox](https://www.youtube.com/watch?v=ZaiA3hWaRzE)
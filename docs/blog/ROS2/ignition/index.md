---
title: Ignition ROS2
tags:
    - ros_ign
---

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

## custom launch

- Run ignition
- Run bridge
- Run rviz


```python
# Bridge
bridge = Node(
    package='ros_ign_bridge',
    executable='parameter_bridge',
    arguments=['/model/vehicle_blue/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
               '/model/vehicle_blue/battery/linear_battery/state@sensor_msgs/msg/BatteryState@ignition.msgs.BatteryState'],
    output='screen'
)
```

!!! tip "copy launch files"
     ```
        install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}
        )
     ```
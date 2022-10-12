---
title: ros2 ign bridge
tags:
    - ignition
    - ros2
---


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
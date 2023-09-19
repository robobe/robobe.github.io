---
title: ignition ros2 bridge
tags:
    - ros2
    - ignition
    - bridge
---

```
sudo apt install ros-humble-ros-ign
```

## diff drive
```bash title="ros2 command"
ros2 topic pub -1 /model/vehicle_blue/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0}}"
```

```bash title="ign command"
ign topic -t "/model/vehicle_green/cmd_vel" -m ignition.msgs.Twist -p "linear: {x: 0.5}, angular: {z: 0.05}"
```

### launch
#### bridge
```python
bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=['/model/vehicle_blue/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
                '/model/vehicle_blue/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
                '/model/vehicle_green/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
                '/model/vehicle_green/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry'],
    parameters=[{'qos_overrides./model/vehicle_blue.subscriber.reliability': 'reliable',
                    'qos_overrides./model/vehicle_green.subscriber.reliability': 'reliable'}],
    output='screen'
)
```

```python
# topic@ros2 msg type@ignition msg type
/model/vehicle_blue/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist
```
---
title: Ignition simulator
tags:
    - simulation
    - gazebo
---
!!! note ""
    All tutorial run on Fortress version
    Newer version `garden` for example move from `ign` to `gz` 
     

# official site

[gazebo sim](https://app.gazebosim.org/dashboard)


|   |   |
|---|---|
| IGN_GAZEBO_RESOURCE_PATH  |   |
| IGN_GAZEBO_SYSTEM_PLUGIN_PATH |  |

```
export IGN_GAZEBO_RESOURCE_PATH=~/projects/ign_tutorial/worlds:~/projects/ign_tutorial/models
```

```
$ ign topic -t "/model/vehicle_blue/cmd_vel" -m ignition.msgs.Twist -p "linear: {x: 0.5}, angular: {z: 0.05}"
```

```
/usr/share/ignition/ignition-gazebo6/worlds
```
## Resource 
[fortress LTS docs](https://gazebosim.org/docs/fortress)
[gazebodoc](https://gazebosim.org/api/sim/6/resources.html)


## Tutorials
- [hello ignition gazebo/sim](hello.md)
- [gui config](gui-control.md)
- [ignition tutorial - building your own robot (gazebosim)](https://github.com/gazebosim/docs/blob/master/fortress/building_robot.md)
- [ignition tutorial - moving the robot (gazebosim)](https://github.com/gazebosim/docs/blob/master/fortress/moving_robot.md)
- [moving robot using ros_gz](moving_robot_ex.md)
- [camera sensor](sensors/camera.md)
- [imu lidar and more (gazebosim)](https://github.com/gazebosim/docs/blob/master/fortress/sensors.md)


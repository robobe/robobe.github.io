---
title: Ardupilot MAVROS minimal example
tags:   
    - ardupilot
    - mavros
---

# Objective
Minimal connection to arducopter with mavros

- Run SITL (without gazebo)
- Run MAVROS
- Get State and change mode

## SITL

```bash title="terminal1"
sim_vehicle.py -v ArduCopter
```

## mavros
```bash title="terminal2"
ros2 run  mavros mavros_node --ros-args -p fcu_url:=udp://:14550@
```

## rqt
```bash title="terminal3"
ros2 run rqt_gui rqt_gui
```

- Load topic_monitor plugin
  - mark `mavros/state` topic
- Load Service Caller plugin
  - select `mavros/set_mode` service
  
![](images/rqt_mavros_topic_service_state_mode.png)

- Set service `custom mode` field to `GUIDED` and call
- Check that `mavros/state` topic changed

![](images/rqt_mavros_topic_service_state_mode_guided.png)
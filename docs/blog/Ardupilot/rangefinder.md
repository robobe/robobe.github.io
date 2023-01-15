---
title: Add rangefinder using SITL and gazebo 
tags:
    - ardupilot
    - rangefinder
---

- R

```bash title="sitl"
sim_vehicle.py -v ArduCopter -f gazebo-iris -A "--defaults /home/user/wasp_ws/src/wasp_bringup/config/copter.parm,/home/user/wasp_ws/src/wasp_bringup/config/gazebo-iris.parm" -I0 -m "--out=127.0.0.1:14552" -m "--load-module graph"
```

![](images/gazebo_sitl_mavproxy_graph_qgc.png)

---
tags:
    - ardupilot
    - gazebo
    - sitl
    
---

# Ardupilot SITL with Gazebo classic
Using ardupilot plugin to connect SITL with gazebo classic
Add Servo gimbal with 3DoF and control it from the SITL.

- Ardupilot Copter ver 4.5
- Gazebo classic ver 11
- Plugin from [ardupilot_gazebo](https://github.com/SwiftGust/ardupilot_gazebo/tree/master)
- Model: iris

!!! note "Plugins"
    There are multiple plugin implementation
    - From gazebo 11 [github](https://github.com/gazebosim/gazebo-classic/blob/gazebo11/plugins/ArduCopterPlugin.cc)
    - From ardupilot github [github](https://github.com/ArduPilot/ardupilot_gazebo)
     

## Demo

```bash title="Running SITL"
./Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris 
```


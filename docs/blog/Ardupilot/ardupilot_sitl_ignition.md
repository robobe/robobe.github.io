---
title: Ardupilot SITL with ignition simulator
tags:
    - ardupilot
    - sitl
    - plugin
---

## Objective
Run SITL and ignition simulation
- ignition fortress
- ardupilot_gazebo (plugin)
- MAVProxy (optional)

### ardupilot_gazebo
- clone `https://github.com/ArduPilot/ardupilot_gazebo.git`
  - ignition-fortress branch
- build check instruction `https://github.com/ArduPilot/ardupilot_gazebo#installation-`

```bash
git clone -b <branchname> <remote-repo-url>
git clone -b ignition-fortress https://github.com/ArduPilot/ardupilot_gazebo.git
```

#### Run
```bash title="terminal1"
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=$HOME/git/ardupilot_gazebo/build:${IGN_GAZEBO_SYSTEM_PLUGIN_PATH}
export IGN_GAZEBO_RESOURCE_PATH=$HOME/git/ardupilot_gazebo/models:$HOME/git/ardupilot_gazebo/worlds:${IGN_GAZEBO_RESOURCE_PATH}

# note remove sky tag from sdf
ign gazebo -v 4 -r iris_arducopter_runway.world
```

#### SITL
```bash title="terminal2 - run SITL"
./arducopter -S --model JSON \
--speedup 1 \
--slave 0 \
--defaults copter.parm,gazebo-iris.parm \
-I0
```

#### MAVProxy
```bash title="mavproxy"
mavproxy.py --master tcp:127.0.0.1:5760

# Arm and takeoff
mode guided
arm throttle
takeoff 5
```

![](images/gazebo.png)

---

# Reference
- [ardupilot_gazebo](https://github.com/ArduPilot/ardupilot_gazebo/tree/ignition-fortress)
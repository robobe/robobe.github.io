---
title: 
tags:
    - ardupilot
    - sitl
    - plugin
    - gazebo
    - garden
---
# Ardupilot SITL with Gazebo simulator

## Objective
Run Ardupilot (copter) SITL and gazebo simulation

- Gazebo garden
- ardupilot_gazebo (plugin)
- MAVProxy


### ardupilot_gazebo


```bash
# Clone ardupilot_gazebo repository
# git subfolder

mkdir ~/git
cd git
# git clone -b <branchname> <remote-repo-url>
git clone https://github.com/ArduPilot/ardupilot_gazebo.git
```

#### Install

[more](https://github.com/ArduPilot/ardupilot_gazebo#installation)

```bash
sudo apt update
sudo apt install libgz-sim7-dev rapidjson-dev
```

```bash
cd aurdupilot_gazebo
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j4
```


#### Run

```bash title="terminal1"
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/git/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}
export GZ_SIM_RESOURCE_PATH=$HOME/git/ardupilot_gazebo/models:$HOME/git/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH}

# note remove sky tag from sdf
gz sim -v 4 -r iris_runway.sdf
```

-r: run simulation on start
-v 4: verbose mode

#### SITL
##### sim_vehicle

```
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console
```

##### Manual
```bash title="terminal2 - run SITL"
./arducopter -S --model JSON \
--speedup 1 \
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

## Basic Worlds and models

|   |   |
|---|---|
| iris_with_ardupilot  | iris_with_standoff, liftdrag and ardupilot plugins  |
| iris_with_gimbal  | iris_with_standoff, gimbal_small_2d, liftdrag and ardupilot plugins  |
| gimbal_small_2d  |  Camera sensor declare on tilt_link |


!!! note 
    In my computer the sky flickering so i comments them

    ```xml
    <scene>
      <ambient>1.0 1.0 1.0</ambient>
      <background>0.8 0.8 0.8</background>
      <!-- <sky></sky> -->
    </scene>
    ```
     

!!! note
    Change model to iris_with_gimbal

    ```xml
    <include>
      <uri>model://iris_with_gimbal</uri>
      <pose degrees="true">0 0 0 0 0 90</pose>
    </include>
    ```


by default camera topic name generate by sensor location

```
/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/tilt_link/sensor/camera/image
```
---

# Reference
- [ardupilot_gazebo](https://github.com/ArduPilot/ardupilot_gazebo)
- [SITL Models](https://github.com/ArduPilot/SITL_Models/tree/master)
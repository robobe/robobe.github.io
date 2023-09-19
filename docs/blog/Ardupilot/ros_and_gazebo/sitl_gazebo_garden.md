---
tags:
    - ardupilot
    - gazebo
    - garden
    - sitl
---

# Using SITL with Gazebo
[ardupilot site](https://ardupilot.org/dev/docs/sitl-with-gazebo.html)


## Install SITL

```
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git

cd ardupilot
./waf configure --board sitl
./waf copter
cd ArduCopter
../Tools/autotest/sim_vehicle.py --console
```

## Build plugin
[Install the ArduPilot Gazebo Plugin](https://ardupilot.org/dev/docs/sitl-with-gazebo.html#install-the-ardupilot-gazebo-plugin)

```bash
git clone https://github.com/ArduPilot/ardupilot_gazebo

```

```bash
cd ardupilot_gazebo
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j4
```

```bash title="set gazebo environment"
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot/gz_ws/src/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot/gz_ws/src/ardupilot_gazebo/models:$HOME/ardupilot/gz_ws/src/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH
```

## Run

!!! note Disabled SKY on my computer

```xml title="iris_runway.sdf"
<scene>
    <ambient>1.0 1.0 1.0</ambient>
    <background>0.8 0.8 0.8</background>
    <!-- <sky></sky> -->
</scene>
```
     

```bash title="gazebo"
gz sim -v4 -r iris_runway.sdf
```

```bash title="sitl"
../Tools/autotest/sim_vehicle.py \
-v ArduCopter \
-f gazebo-iris \
--model JSON \
--console

```

```bash title="console"
mode guided
arm throttle
takeoff 5
```

![](../images/sitl_gazebo_garden.png)
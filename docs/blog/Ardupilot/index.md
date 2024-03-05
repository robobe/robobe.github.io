---
title: Ardupilot
tags:
    - ardupilot
    - 
---

# Install dev environment
- Clone
- Sync submodules
- Build Docker
- waf it.

```bash title="clone"
git clone https://github.com/ArduPilot/ardupilot.git
```

```bash title="sync sub modules"
cd ardupilot
./Tools/gittools/submodule-sync.sh 
```

```bash title="build and run docker"
# from ardupilot folder
docker build . -t ardupilot
docker run --rm -it -v `pwd`:/ardupilot ardupilot:latest bash
```

```bash title="waf from docker"
#./waf list_boards
./waf configure --board=sitl
./waf copter
```

```bash title="other waf command"
./waf distclean: remove build folder and data
./waf build --upload: build and upload to board
```

---

## Switch version

```bash title="list all branch's"
git branch -v -a
```

```bash title="switch"
# git switch -c <local branch name> <remote branch without the remote prefix>
git switch -c Copter-4.4 origin/Copter-4.4
```

```bash title="build"
# from ardupilot folder
./Tools/gittools/submodule-sync.sh
# run docker
./waf distclean
./waf configure --board=sitl
./waf copter
```


---

## Tutorial
- [Run SITL and Gazebo simulator](ardupilot_sitl_ignition.md)

# Reference
- [ros-gz-rover](https://github.com/srmainwaring/ros_gz_rover)
- [Ardupilot SITL models for gazebo and ignition](https://github.com/ArduPilot/SITL_Models)
- [BlueROV2 in Ignition Gazebo](https://github.com/srmainwaring/bluerov2_ignition)
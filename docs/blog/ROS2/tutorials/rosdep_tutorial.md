---
tags:
    - rosdep
    - ros2
    - package manager
    - project settings
---
# rosdep tutorial
rosdep is a command-line tool for installing system dependencies
rosdep is ROS’s dependency management utility that can work with ROS packages and external libraries.


## usage
### first use
```bash
sudo rosdep init
rosdep update
```

**rosdep init** create folder `rosdep` under `/etc/ros`  
**rosdep update** download index files to local cache  

index file url set `/etc/ros/rosdep/sources.list.d` folder

for example python packages from pip defined at `https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/python.yaml`

index file for ros distro download from distro location that config `https://raw.githubusercontent.com/ros/rosdistro/master/index-v4.yaml`

### run
!!! note "update"
    run `rosdep update` to update cache index
     
rosdep will check for `package.xml` files in its path or for a specific package and find the rosdep keys stored within

```
rosdep install --simulate --from-paths ~/workspaces/project/src --ignore-src
```

- **--form-path**: specifies the path to check for package.xml files to resolve keys for
- **y**: means to default yes to all prompts from the package manager to install without prompts
- **--ignore-src**: means to ignore installing dependencies, even if a rosdep key exists, if the package itself is also in the workspace


## demo
install python package using rosdep  

- add package name to `package.xml` depend tag  
      1. check python.yaml for package correct name  
- run rosdep `install` 

```xml title="package.xml"
<depend>pynput-pip</depend>
```

```bash title="terminal"
rosdep install --from-paths ~/ros2_ws/src/camera_calibration_gazebo --ignore-src

```

!!! note "check python package naming"
    browse to `https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/python.yaml`
     
    ```yaml
    pynput-pip:
        debian:
            pip:
            packages: [pynput]
        ubuntu:
            pip:
            packages: [pynput]
    ```


## resolve
resolve <rosdeps> to system dependencies

```bash title="install from pip"
rosdep resolve pynput-pip
#pip
pynput
```

```bash title="install from apt"
rosdep resolve python-opencv
#apt
python-opencv

```



---

## Reference
- [Managing Dependencies with rosdep](https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html)
- [Using Python Packages with ROS 2](https://docs.ros.org/en/humble/How-To-Guides/Using-Python-Packages.html)
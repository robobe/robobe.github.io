---
title: Package.xml hello
description: Basic ROS2 launch file
date: "2022-05-16"
banner: ../ros2.png
tags:
    - ros2
    - package.xml
    - 101
---


```xml title="package.xml" linenums="1" hl_lines="3"
<exec_depend>rclpy</exec_depend>
<exec_depend>std_msgs</exec_depend>
<depend>example_interfaces</depend>
<test_depend>ament_copyright</test_depend>
...
```

## rosdep
```bash title="install"
sudo apt install python3-rosdep
```

```bash title="init"
sudo rosdep init
rosdep update
```

### command
#### resolve
```bash
rosdep resolve example_interfaces
#apt
ros-foxy-example-interfaces
```

#### install
```bash title="install package dependencies"
# from w.s root run
rosdep install --from-paths src/basic --ignore-src -r -y
# Command result
executing command [sudo -H apt-get install -y ros-foxy-example-interfaces]

```

---

# Resources
- [rep-149 Package Manifest Format Three Specification](https://ros.org/reps/rep-0149.html)
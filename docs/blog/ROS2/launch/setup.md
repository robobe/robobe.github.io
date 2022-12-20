---
title: launch files tutorial package setup
tags:
    - ros2
    - launch
    - 101
---

Setup launch tutorial pkg and write minimal launch file

```bash
ros2 pkg create pkg_launch_tutorial --build_type ament_cmake
```

!!! note ""
    - Remove  `include` and `src`
    - Create `launch` sub folder


```c title="CMakeLists.txt"
// Add to CMakeLists.txt
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
```

!!! tip ""
    Check [my git](https://github.com/robobe/ros2_launch_tutorial)
     
     
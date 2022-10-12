---
title: ROS2 launch system
tags:
    - ros2
    - launch
---

# tips
## copy launch folder
### cmake

```c
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
```

---

# Reference
- [Architecture of launch](https://github.com/ros2/launch/blob/humble/launch/doc/source/architecture.rst)
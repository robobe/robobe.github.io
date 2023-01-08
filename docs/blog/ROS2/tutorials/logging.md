---
title: ROS2 logging
tags:
    - ros2
    - tutorial
    - logging
---

[ros2 wiki logging](https://docs.ros.org/en/humble/Tutorials/Demos/Logging-and-logger-configuration.html)

!!! note "colorize logging"
    Add environment variable to `.bashrc`
    ```
    export RCUTILS_COLORIZED_OUTPUT=1
    ```
     
## logging control demo

- control node log level
- Set logging format
  
```python title="very simple node with logging"
--8<-- "/home/user/ros2_ws/src/pkg_launch_tutorial/pkg_launch_tutorial/minimal_node.py"
```

```python title="node.launch.py" linenums="1" hl_lines="3 6"
--8<-- "/home/user/ros2_ws/src/pkg_launch_tutorial/launch/node.launch.py"
```

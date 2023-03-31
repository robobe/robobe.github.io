---
tags:
    - ros2
    - tutorial
    - logging
---
# ROS2 logging
ROS2 logging control and config

## config
[ros2 wiki logging](https://docs.ros.org/en/humble/Tutorials/Demos/Logging-and-logger-configuration.html)

### environment variables
- RCUTILS_COLORIZED_OUTPUT
- ROS_LOG_DIR
- RCUTILS_LOGGING_USE_STDOUT
- RCUTILS_LOGGING_BUFFERED_STREAM
- RCUTILS_CONSOLE_OUTPUT_FORMAT


###  RCUTILS_CONSOLE_OUTPUT_FORMAT
```
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity} {time}] [{name}]: {message} ({function_name}() at {file_name}:{line_number})"
```


### RCUTILS_COLORIZED_OUTPUT
!!! note "colorize logging"
    Add environment variable to `.bashrc`
    ```
    export RCUTILS_COLORIZED_OUTPUT=1
    ```
     
---

## Control

### throttled
```python title="throttled"
node.get_logger().error("log every sec", throttle_duration_sec=1)
```

### one time
```python title="log only first time"
node.get_logger().info(f'log only once', once=True)
```

### skip
```python title="log, skip first time"
node.get_logger().info(f'log only once', skip_first=True)
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


---

!!! note "log external config"
    ```
    ros2 run some_package some_ros_executable --ros-args --log-config-file some_log.config
    ```
    ROS2 logging base on spdlog  
    for know spdlog not support external config
    
    [check humble code](https://github.com/ros2/rcl_logging/blob/humble/rcl_logging_spdlog/src/rcl_logging_spdlog.cpp#L69)
     
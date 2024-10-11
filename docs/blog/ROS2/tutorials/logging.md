---
tags:
    - ros2
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


#####  RCUTILS_CONSOLE_OUTPUT_FORMAT
```
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity} {time}] [{name}]: {message} ({function_name}() at {file_name}:{line_number})"
```


##### RCUTILS_COLORIZED_OUTPUT
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
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class LogDemo(Node):

    def __init__(self):
        super().__init__('log_demo')
        self.get_logger().debug("------- debug")
        self.get_logger().info("------- info")
        self.get_logger().warn("------- warn")
        self.get_logger().error("------- error")


def main(args=None):
    rclpy.init(args=args)
    node = LogDemo()
    rclpy.spin_once(node, timeout_sec=1)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

```bash title="control log level"
ros2 run rome_demos_py log_demo.py --ros-args --log-level ERROR
```

#### launch file

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo
from launch.substitutions import EnvironmentVariable

def generate_launch_description():
    ld = LaunchDescription()

    log_level = EnvironmentVariable("LOG_LEVEL", default_value="INFO")

    tracker =  Node(
            package='rome_demos_py',
            namespace='',
            executable='log_demo.py',
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            respawn=True
        )
    
    log = LogInfo(msg=["----------", log_level])

    ld.add_action(log)
    ld.add_action(tracker)
    return ld
```

---

!!! note "log external config"
    ```
    ros2 run some_package some_ros_executable --ros-args --log-config-file some_log.config
    ```
    ROS2 logging base on spdlog  
    for know spdlog not support external config
    
    [check humble code](https://github.com/ros2/rcl_logging/blob/humble/rcl_logging_spdlog/src/rcl_logging_spdlog.cpp#L69)
     
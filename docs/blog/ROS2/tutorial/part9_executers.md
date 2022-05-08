---
title: Part9 - executers
description: hello ROS2 executers
date: "2022-05-06"
banner: ../ros2.png
tags:
    - python
    - executers
---

```python title="minimal_timer"
import rclpy
from rclpy.node import Node

class MinimalTimer(Node):
    def __init__(self):
        super().__init__("simple_timer")
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        self.get_logger().info(f'timer tick: {self.i}')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    minimal_timer = MinimalTimer()
    rclpy.spin(minimal_timer)
    minimal_timer.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

```

```python title=minimal_sta"
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

class MinimalTimer(Node):
    def __init__(self):
        super().__init__("simple_timer")
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        self.get_logger().info(f'timer tick: {self.i}')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    minimal_timer = MinimalTimer()
    ste = SingleThreadedExecutor()
    ste.add_node(minimal_timer)
    ste.spin()
    minimal_timer.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

```

---

# Reference
- [test_executor](https://github.com/ros2/rclpy/blob/foxy/rclpy/test/test_executor.py)
- [Execution and Callbacks¶](https://docs.ros2.org/foxy/api/rclpy/api/execution_and_callbacks.html)
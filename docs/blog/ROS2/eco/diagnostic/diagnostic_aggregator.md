---
tags:
    - ros2
    - diagnostic
    - aggregator
---

```yaml
analyzers:
  ros__parameters:
    path: Aggregation
    arms:
      type: diagnostic_aggregator/DiscardAnalyzer
      path: hb_data
      startswith: [ '/hb' ]
```

```bash
ros2 run diagnostic_aggregator aggregator_node --ros-args --params-file /workspaces/rome_ws/src/rome_demos_py/config/analyzer.yaml
```

```python
#!/usr/bin/env python3

from random import random

from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
import rclpy
from rclpy.clock import ROSClock
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

PKG = 'diagnostic_aggregator'


class DiagnosticTalker(Node):

    def __init__(self):
        super().__init__('diagnostic_talker')
        self.pub = self.create_publisher(DiagnosticArray,
                                         '/diagnostics',
                                         qos_profile_system_default)
        timer_period = 1.0
        self.tmr = self.create_timer(timer_period, self.timer_callback)

        self.array = DiagnosticArray()
        self.array.status = [
            # Motors
            DiagnosticStatus(level=DiagnosticStatus.OK,
                             name='/hb', message='OK'),
        ]

    def timer_callback(self):
        self.array.header.stamp = ROSClock().now().to_msg()
        self.pub.publish(self.array)

def main(args=None):
    rclpy.init(args=args)

    node = DiagnosticTalker()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
```

--- 

## Reference
- [Creating a Diagnostic Analyzer](http://wiki.ros.org/diagnostics/Tutorials/Creating%20a%20Diagnostic%20Analyzer)
- [ros2 plugin](https://github.com/issaiass/ros2_examples/tree/main/src/polygon_plugins)
- [The diagnostic_aggregator package](https://github.com/ros/diagnostics/tree/ros2/diagnostic_aggregator)
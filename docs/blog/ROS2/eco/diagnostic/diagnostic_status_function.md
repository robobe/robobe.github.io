---
tags:
    - ros2
    - diagnostics
    - tutorials
    - diagnostic_status
---

# DiagnosticStatus as function

Implement `DiagnosticStatus` as function that register to DiagnosticUpdater

## Demo
Simple ROS2 python node that use diagnostic_updater to send diagnostic message

- DiagnosticStatusWrapper
- DiagnosticStatus
- diagnostic_updater

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticStatus
import diagnostic_updater

def dummy_diagnostic(stat: diagnostic_updater.DiagnosticStatusWrapper):
   stat.message ="message dummy_diagnostic"
   stat.level = DiagnosticStatus.WARN
   stat.name = "dummy_diagnostic"

   return stat

class MyNode(Node):
    def __init__(self):
        node_name="minimal"
        super().__init__(node_name)
        updater = diagnostic_updater.Updater(self)
        updater.add("Function updater", dummy_diagnostic)
        self.get_logger().info("Hello ROS2")

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

- **dummy_diagnostic** method get one argument stat of type **diagnostic_updater.DiagnosticStatusWrapper**
DiagnosticStatusWrapper is a derived class of
diagnostic_msgs.msg.DiagnosticStatus that provides a set of convenience
methods.

- diagnostic_updater use **add** function to register diagnostic method 
    ```
    add(name, function)
    ```
---

```bash title="topic diagnostics"
ros2 topic echo /diagnostics 
header:
  stamp:
    sec: 1727264131
    nanosec: 26442474
  frame_id: ''
status:
- level: "\x01"
  name: 'minimal: dummy_diagnostic'
  message: message dummy_diagnostic
  hardware_id: ''
  values: []
```
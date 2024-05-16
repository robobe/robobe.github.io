---

tags:
    - ros2
    - rclpy
    - diagnostic
    - tutorials
---

# Remove Diagnostic tasks

Using `DiagnosticUpdater` `removeByName` method

```python
import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import (
    DiagnosticTask, 
    DiagnosticStatusWrapper, 
    Updater
)

class HeartbeatStatus(DiagnosticTask):
    def __init__(self, name):
        super().__init__(name)
        self.counter = 0


    def run(self, stat: DiagnosticStatusWrapper):
        stat.summary(DiagnosticStatus.OK, "Normal")
            
        stat.add("counter", str(self.counter))
        self.counter += 1
        stat.add("more data", "data")
        return stat

class MyNode(Node):
    def __init__(self):
        node_name="minimal"
        super().__init__(node_name)
        self.dia_updater = Updater(self, period=0.5)
        self.hb_diag = HeartbeatStatus("Demo HB")
        self.dia_updater.add(self.hb_diag)
        self.timer = self.create_timer(2.0, self.handler)

    def handler(self):
        self.timer.cancel()
        self.dia_updater.removeByName("Demo HB")
    

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```


```bash
ros2 topic echo /diagnostics

#
---
header:
  stamp:
    sec: 1715662545
    nanosec: 50457855
  frame_id: ''
status:
- level: "\0"
  name: 'minimal: Demo HB'
  message: Normal
  hardware_id: ''
  values:
  - key: counter
    value: '1'
  - key: more data
    value: data
---
header:
  stamp:
    sec: 1715662546
    nanosec: 51780763
  frame_id: ''
status: []
---
```

![rqt_runtime_monitor_stale](images/rqt_runtime_monitor_stale.png)
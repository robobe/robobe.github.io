---

tags:
    - ros2
    - diagnostics
    - tutorials
---

# Diagnostic Composite Task

Demo using `CompositeDiagnosticTask` to combine multiple Diagnostic task (`FunctionDiagnosticTask`) to same DiagnosticStatus message

```python
import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import (
    DiagnosticTask, 
    DiagnosticStatusWrapper, 
    Updater, 
    FrequencyStatus,
    FrequencyStatusParam,
    HeaderlessTopicDiagnostic,
    TimeStampStatus,
    Heartbeat,
    CompositeDiagnosticTask,
    FunctionDiagnosticTask
)


class MyNode(Node):
    def __init__(self):
        node_name="minimal"
        super().__init__(node_name)
        self.data_to_diagnostic = 10
        self.diagnostic_updater = Updater(self)
        lower = FunctionDiagnosticTask("lower check", self.check_lower_bound)
        upper = FunctionDiagnosticTask("upper check", self.check_upper_bound)
        comp_task = CompositeDiagnosticTask("range demo")
        comp_task.addTask(lower)
        comp_task.addTask(upper)
        self.diagnostic_updater.add(comp_task)
        self.get_logger().info("ros2 CompositeDiagnosticTask and FunctionDiagnosticTask demo")

    
    def check_lower_bound(self, state: DiagnosticStatusWrapper):
        if self.data_to_diagnostic < 100:
            state.summary(DiagnosticStatus.ERROR, "data out of range")
        else:
            state.summary(DiagnosticStatus.OK, "lower ok")
        return state


    def check_upper_bound(self, state: DiagnosticStatusWrapper):
        state.summary(DiagnosticStatus.OK, "upper ok")
        return state

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

```bash title="all ok"
ros2 topic echo /diagnostics 

#
---
header:
  stamp:
    sec: 1715828741
    nanosec: 47712579
  frame_id: ''
status:
- level: "\0"
  name: 'minimal: range demo'
  message: lower ok; upper ok
  hardware_id: ''
  values: []
---
```

```bash title="one of the function diagnostic alert"
---
header:
  stamp:
    sec: 1715829064
    nanosec: 124827790
  frame_id: ''
status:
- level: "\x02"
  name: 'minimal: range demo'
  message: data out of range
  hardware_id: ''
  values: []
---
```
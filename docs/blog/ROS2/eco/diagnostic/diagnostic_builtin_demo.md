---

tags:
    - ros2
    - rclpy
    - diagnostic
    - tutorials
---

# Using Predefine Tasks

- FrequencyStatus
- TimeStampStatus
- Heartbeat



## FrequencyStatus
A diagnostic task that monitors the frequency (in hz) of an event.


### Demo

- FrequencyStatus
- FrequencyStatusParam
- Updater

Monitor tick call between min and max frequency

!!! tip Using updater add method
    Updater add method has two variant
    - one argument, Register class run method, and got the diagnostic name from the class
    - Tow arguments: `add(name, func)` : Register function and not a DiagnosticTask
      
    ```python
    def add(self, *args):
      ...
      if len(args) == 1:
          task = DiagnosticTaskVector.DiagnosticTaskInternal(
              args[0].getName(), args[0].run)
      elif len(args) == 2:
          task = DiagnosticTaskVector.DiagnosticTaskInternal(
              args[0], args[1])
    ```


In the demo we declare timer that tick The DiagnosticTask at 10 hz

```python
import rclpy
from rclpy.node import Node
from diagnostic_updater import FrequencyStatus, Updater, FrequencyStatusParam

class MyNode(Node):
    def __init__(self):
        node_name="node_name"
        super().__init__(node_name)
        updater = Updater(self)
        updater.hwid = "hwid"
        self.create_timer(1/10, self.timer_handler)
        frequency_params = FrequencyStatusParam(
            {'min': 9.0, 'max': 11.0},  # Expected frequency range (Hz)
            0.1,  # Tolerance (acceptable deviation)
            10,   # Window size (number of events to average over)
        )

        # Add the FrequencyStatus task to the updater
        self.frequency_task = FrequencyStatus(frequency_params, name="check tick frequency")
        updater.add(self.frequency_task)
        self.get_logger().info("Hello ROS2")

    def timer_handler(self):
        self.frequency_task.tick()

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## TimeStampStatus

Diagnostic task to monitor the interval between events.

The task monitor the message time stamp header or other source of time, its compare between the stamp to current time

### Demo
Simulate header time stamp by timer event and decrees 1sec duration from this time

The diagnostic task create **error** with `Timestamps too far in past seen.` message

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from diagnostic_updater import TimeStampStatus, TimeStampStatusParam, Updater

class MyNode(Node):
    def __init__(self):
        node_name="node_name"
        super().__init__(node_name)
        updater = Updater(self)
        updater.hwid = "hwid"
        self.create_timer(1, self.timer_handler)
        time_params = TimeStampStatusParam(
            min_acceptable=0.1,
            max_acceptable=0.5
        )

        # Add the FrequencyStatus task to the updater
        self.task = TimeStampStatus(time_params, name="check time stamp")
        updater.add(self.task)
        self.get_logger().info("Hello ROS2")

    def timer_handler(self):
        d = Duration(seconds=1)
        current_time = self.get_clock().now()
        self.task.tick(current_time-d)

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Heartbeat

Diagnostic task to monitor whether a node is alive.
This diagnostic task always reports as OK and 'Alive' when it runs

### Demo

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from diagnostic_updater import Heartbeat, Updater

class MyNode(Node):
    def __init__(self):
        node_name="node_name"
        super().__init__(node_name)
        updater = Updater(self)
        updater.hwid = "hwid"
        self.task = Heartbeat()
        updater.add(self.task)
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

```bash

header:
  stamp:
    sec: 1727289836
    nanosec: 399848537
  frame_id: ''
status:
- level: "\0"
  name: 'node_name: Heartbeat'
  message: Alive
  hardware_id: hwid
  values: []
```
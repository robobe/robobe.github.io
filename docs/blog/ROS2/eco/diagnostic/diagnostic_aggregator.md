---
tags:
    - ros2
    - diagnostic
    - aggregator
---
# diagnostic_agg

In ROS 2, the **diagnostic_agg** package is used to aggregate and analyze diagnostic messages published by various nodes in the system

### Install

```
sudo apt install ros-humble-diagnostic-aggregator
```

!!! note "DiagnosticStatus"
    - level: "\0"
    name: 'node_name: Heartbeat'
    message: Alive
    hardware_id: hwid
    values: []


!!! tip "GenericAnalyzer"
    All Filter/Matching Fields can be array of strings


    - type: This is the class name of the analyzer, used to load the correct plugin type.
    - path: All diagnostic items analyzed by the Analyzer will be under "Base Path/My Path".
    - contains Any item that contains these values
    - startswith Item name must start with this value
    - ?? name Exact name match ??
    - ?? expected Exact name match, will warn if not present??
    - regex Regular expression (regex) match against name 

    
```yaml
analyzers:
  ros__parameters:
    path: Aggregation
    message_staleness:
      type: diagnostic_aggregator/GenericAnalyzer
      path: heartbeat
      startswith: [ 'node_name: Heartbeat' ]
```

### Usage
```bash
#ros2 run diagnostic_aggregator diagnostic_aggregator --ros-args --params-file path/to/agg_config.yaml
ros2 run diagnostic_aggregator aggregator_node --ros-args --params-file config/analyzer.yaml
```

---

## Demo
Config aggregation to alert if diagnosticStatus missing

```python title="node with heartbeat diagnostic"
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

```yaml title="/diagnostic"
header:
  stamp:
    sec: 1727325239
    nanosec: 655198037
  frame_id: ''
status:
- level: "\0"
  name: 'node_name: Heartbeat'
  message: Alive
  hardware_id: hwid
  values: []
```

### aggregator configuration

```yaml
analyzers:
  ros__parameters:
    node_hb:
      type: diagnostic_aggregator/GenericAnalyzer
      path: master_caution
      startswith: [ 'node_name: Heartbeat' ]
```

#### aggregator output
- group status
- item with group prefix

aggregator output when HB ok

```yaml title="diagnostic_agg"
header:
  stamp:
    sec: 1727325395
    nanosec: 533586972
  frame_id: ''
status:
- level: "\0"
  name: /master_caution
  message: OK
  hardware_id: ''
  values:
  - key: 'node_name: Heartbeat'
    value: Alive
- level: "\0"
  name: '/master_caution/node_name: Heartbeat'
  message: Alive
  hardware_id: hwid
  values: []
```

aggregator output when no HB 
- `master_caution` group alert with error
- message `Stale` meaning that some group item missing
- the hb status itself switch to error
- default timeout: 5sec
```yaml
header:
  stamp:
    sec: 1727325557
    nanosec: 534785208
  frame_id: ''
status:
- level: "\x03"
  name: /master_caution
  message: Stale
  hardware_id: ''
  values:
  - key: 'node_name: Heartbeat'
    value: Alive
- level: "\x03"
  name: '/master_caution/node_name: Heartbeat'
  message: Alive
  hardware_id: hwid
  values: []
```

!!! tip "timeout"
    Add **timeout** to yaml file
    ```yaml
    analyzers:
        ros__parameters:
            node_hb:
            type: diagnostic_aggregator/GenericAnalyzer
            path: master_caution
            startswith: [ 'node_name: Heartbeat' ]
            timeout: 1.0
    ```
     

--- 

## Reference
- [Creating a Diagnostic Analyzer](http://wiki.ros.org/diagnostics/Tutorials/Creating%20a%20Diagnostic%20Analyzer)
- [ros2 plugin](https://github.com/issaiass/ros2_examples/tree/main/src/polygon_plugins)
- [The diagnostic_aggregator package](https://github.com/ros/diagnostics/tree/ros2/diagnostic_aggregator)
---
tags:
    - ros2
    - rclpy
    - namespace
    - 
---

## node rename

```bash
ros2 run basic_pkg node1
```

```bash
ros2 run basic_pkg node1 --ros-args -r __node:=node_1
```

```python title="get node name from code"
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self) -> None:
        node_name = "basic_node"
        super().__init__(node_name)
        self.get_logger().info(f"node name: {self.get_name()}")
```

---

## node namespace


```bash
 ros2 run basic_pkg node1 --ros-args -r __node:=node_1 -r __ns:=/custom_ns

```


```python
class MyNode(Node):
    def __init__(self) -> None:
        node_name = "basic_node"
        super().__init__(node_name)
        self.get_logger().info(f"node name: {self.get_name()}")
        self.get_logger().info(f"node namespace: {self.get_namespace()}")
```

```bash
[INFO] [1712818814.485948867] [custom_ns.node_1]: node name: node_1
[INFO] [1712818814.486168820] [custom_ns.node_1]: node namespace: /custom_ns
```
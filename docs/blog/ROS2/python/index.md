---
title: ROS2 Python 
tags:
    - ros2
    - python
---

## Template

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        node_name="minimal"
        super().__init__(node_name)
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

```python title="setup.py"
# copy launch files to install folder
data_files=[
        (os.path.join('share', package_name, "launch"), glob('launch/*.launch.py')),
]

# entry points
entry_points={
        'console_scripts': [
            "minimal_node=py_tutorial_pkg.minimal_node:main",
        ]
```
---

# Tutorial
- [pub sub namespace and remapping](pub_sub_ns_remapping.md)
- [Test with pytest and colcon](test_demo.md)

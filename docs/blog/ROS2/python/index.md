---
title: ROS2 Python 
tags:
    - ros2
    - python
---

## Minimal Node
Minimal python node Template

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

---

## Setup tips

- Copy none python files to `share` folders
- Map entry points to python modules
- Copy sub packages folder to `install\<>\lib\ \site-packages` folder


### copy none python files
```python
# copy launch files to install folder
data_files=[
        (os.path.join('share', package_name, "launch"), glob('launch/*.launch.py')),
]
```

### Entry points
```python title="map entry points"
# entry points
entry_points={
        'console_scripts': [
            "minimal_node=py_tutorial_pkg.minimal_node:main",
        ]
```

### sub folders / packages
- Add `__init__.py` to sub folder 
- import `find_packages` from `setuptools`
- use `find_packages()` method in `setup`


```python linenums="1" hl_lines="4"
setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    ...
)
```



---

# Tutorials
- [pub sub namespace and remapping](pub_sub_ns_remapping.md)
- [Test with pytest and colcon](test_demo.md)

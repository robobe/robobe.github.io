---
tags:
    - ros2
    - parameter
    - declarative
    - yaml
    - generate
---
# generate_parameter_library
Generate C++ or Python code for ROS 2 parameter declaration, getting, and validation using declarative YAML. The generated library contains a C++ struct with specified parameters. Additionally, dynamic parameters and custom validation are made easy.
[More](https://github.com/PickNikRobotics/generate_parameter_library/tree/main)

---

## Demo: 
Simple python package base on ament_cmake
Generate `simple.yaml` parameter declaration to class that usage by the node.

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from param_demo.simple_parameter import simple

class MyNode(Node):
    def __init__(self):
        super().__init__("Simple")
        self.param_listener = simple.ParamListener(self)
        self.params = self.param_listener.get_params()
        self.get_logger().info(f"---float param: {self.params.float_demo}") 
        self.get_logger().info(f"---p param: {self.params.p}") 

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

```yaml
simple:
  float_demo:
    type: double
    default_value: 0.1
    description: "Test scientific notation"
  p:
    type: double
    default_value: 1.0
    description: "proportional gain term"
    validation:
      gt_eq<>: [0.5]
```

```bash title="CMakeLists.txt"
cmake_minimum_required(VERSION 3.16)
project(param_demo)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(generate_parameter_library REQUIRED)

generate_parameter_module(
  simple_parameter # cmake target name for the parameter library
  param_demo/simple.yaml
  
)

set (FILES
  param_demo/simple.py
)

install(PROGRAMS
    ${FILES}
DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

ament_package()
```
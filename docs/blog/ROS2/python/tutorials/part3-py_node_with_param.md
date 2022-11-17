---
title: Part3 - Simple python Node with parameter
description: Basic ROS2 node with parameter, declare, read with type, set from command line or using yaml file, get and set from command line or gui
date: "2022-04-05"
banner: ../ros2.png
tags:
    - ros2
    - param
    - parameter
---

# Objective 

- Declare parameter
- Manage params from cli
- Set node params with launch file
  
## Code example

```python
import rclpy
from rclpy.node import Node

class TestParams(Node):

    def __init__(self):
        super().__init__('test_params_rclpy')

        self.declare_parameter('my_str')
        self.declare_parameter('my_int', value=10)
        self.declare_parameter('my_double_array')

        timer_period = 2
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        my_param = self.get_parameter('my_str').get_parameter_value().string_value
        my_param_int = self.get_parameter("my_int").get_parameter_value().integer_value
        self.get_logger().info(f"Hello {my_param}! with int data: {my_param_int}")

def main(args=None):
    rclpy.init(args=args)
    node = TestParams()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
```

---

## Manage params from cli

### Run node with param
!!! Note
    Run node with arguments from CLI
    `--ros-args -p <param_name>:=<param_value>`

```bash
ros2 run basic simple_param --ros-args -p my_str:=world
# Result
[INFO] [1649226981.611616597] [test_params_rclpy]: Hello world! with int data: 10
[INFO] [1649226983.604038327] [test_params_rclpy]: Hello world! with int data: 10
```

### Manage params from cli

```bash
# list params from all running nodes
ros2 param list
# Result 
/simple_params:
  my_double_array
  my_int
  my_str
  use_sim_time

```

### YAML file
```yaml  title="simple.yaml" linenums="1" hl_lines="2"
simple_params:
  ros__parameters:
    my_str: "world from yaml"
    my_int: 100
    my_double_array: [1.0, 2.0, 3.0]
```

!!! Warning
    `ros__parameters` with double underline

#### Run with yaml
```bash title="terminal1"
ros2 run basic simple_param --ros-args --params-file /home/user/dev_ws/src/basic/config/simple.yaml
[INFO] [1649227913.033306747] [simple_params]: Hello world from yaml! with int data: 100
[INFO] [1649227915.025466940] [simple_params]: Hello world from yaml! with int data: 100
```

#### Params yaml and launch file
- place `yaml` file in `config` syb folder
- copy `config` folder to output folder using `setup.py`

```python title="yaml copy" linenums="1" hl_lines="5"
data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
    ],
```
```python title="simple_param_yaml.launch.py"  linenums="1" hl_lines="14"
# with yaml file
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('basic'),
        'config',
        'simple.yaml'
        )
        
    node=Node(
        name="simple_params",
        package = 'basic',
        executable = 'simple_param',
        parameters = [config]
    )
    ld.add_action(node)
    return ld
```

!!! Warning
    The name argument in the `launch` Node object must be the same in the param yaml file


```python title="simple_param.launch.py"  linenums="1" hl_lines="12-16"

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    ld = LaunchDescription()
        
    node=Node(
        name="simple_params",
        package = 'basic',
        executable = 'simple_param',
        parameters = [
            {"my_str": "hello from launch"},
            {"my_int": 1000},
            {"my_double_array": [1.0, 10.0]}
        ]
    )
    ld.add_action(node)
    return ld
```




---


# References
- [rclpy Params Tutorial – Get and Set ROS2 Params with Python](https://roboticsbackend.com/rclpy-params-tutorial-get-set-ros2-params-with-python/)
- [ROS2 YAML For Parameters](https://roboticsbackend.com/ros2-yaml-params/)
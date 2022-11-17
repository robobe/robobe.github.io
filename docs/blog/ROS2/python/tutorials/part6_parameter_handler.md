---
title: Part6 - Simple parameter handler
description: ROS2 parameters handler
date: "2022-04-08"
tags:
    - ros2
    - parameters
---

# Objective
- Update node param
  - from cli
  - from code


## Code

```python
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

class TestParams(Node):

    def __init__(self):
        super().__init__('simple_params')

        self.declare_parameter('my_str')
        self.declare_parameter('my_int', value=10)
        self.declare_parameter('my_double_array')
        self.add_on_set_parameters_callback(self.parameter_callback)
        timer_period = 2
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def parameter_callback(self, params):
        print(type(params))
        for param in params:
            self.get_logger().info(param.name)
            self.get_logger().info(str(param.value))
            self.get_logger().info(str(param.type_))
        return SetParametersResult(successful=True)

    def timer_callback(self):
        my_param = self.get_parameter('my_str').get_parameter_value().string_value
        my_param_int = self.get_parameter("my_int").get_parameter_value().integer_value
        my_param_array = self.get_parameter("my_double_array").get_parameter_value().double_array_value
        self.get_logger().info(f"Hello {my_param}! with int data: {my_param_int}")
        self.get_logger().info(str(my_param_array))

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

## Run
### List

```bash
ros2 param list
/simple_params:
  my_double_array
  my_int
  my_str
  use_sim_time
```

### get / set
```bash title="get"
# my_str
ros2 param get /simple_params my_str
Parameter not set.

# my_int
ros2 param get /simple_params my_int
Integer value is: 10
```

```bash title="set"
# my_int
ros2 param set /simple_params my_int 20
# result
Set parameter successful

# output log from node callback param function (name, value, type)
[INFO] [1649395615.004278881] [simple_params]: my_int
[INFO] [1649395615.004490765] [simple_params]: 20
[INFO] [1649395615.004679396] [simple_params]: Type.INTEGER

# my_str
ros2 param set /simple_params my_str "world world"
# result
Set parameter successful

# output log from node callback param function (name, value, type)
[INFO] [1649395751.117007430] [simple_params]: my_str
[INFO] [1649395751.117735309] [simple_params]: world world
[INFO] [1649395751.118404909] [simple_params]: Type.STRING

# my_double_array
ros2 param set /simple_params my_double_array "[1.0, 2.0, 3.0]"
# result
Set parameter successful

# output log from node callback param function (name, value, type)
[INFO] [1649396032.647319212] [simple_params]: my_double_array
[INFO] [1649396032.648079309] [simple_params]: array('d', [1.0, 2.0, 3.0])
[INFO] [1649396032.648748882] [simple_params]: Type.DOUBLE_ARRAY

```


## Dump / Load
```bash title="dump"
#ros2 param dump /simple_params --output-dir /tmp
# ros2 param dump /simple_params --print
ros2 param dump /simple_params
# Result
Saving to:  ./simple_params.yaml
```

```bash title="load"
ros2 param load /simple_params simple_params.yaml
# Result 
Set parameter my_double_array successful
Set parameter my_int successful
Set parameter my_str successful
Set parameter use_sim_time successful
```

!!! Note
    `ros2 param load` run the same `param callback` function like `param set`

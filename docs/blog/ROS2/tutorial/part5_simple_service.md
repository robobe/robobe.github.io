---
title: Part5 - Simple Service
description: Basic ROS2 Service
date: "2022-04-05"
banner: ../ros2.png
tags:
    - ros2
    - service
    - 101
---

# Service

```python title="simple_service" linenums="1" hl_lines="9"
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(SetBool, 'echo_service', self.echo_callback)

    def echo_callback(self, request, response: SetBool.Response):
        self.get_logger().info(str(type(request)))
        self.get_logger().info(str(type(response)))
        self.get_logger().info("Incoming request")
        response.success = True
        response.message = "success"
        return response


def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Usage

```bash title="list"
ros2 service list
# Result
/echo_service
/minimal_service/describe_parameters
/minimal_service/get_parameter_types
/minimal_service/get_parameters
/minimal_service/list_parameters
/minimal_service/set_parameters
/minimal_service/set_parameters_atomically

```

```bash
ros2 service type /echo_service
# Result
std_srvs/srv/SetBool
```

```bash title="show interface"
ros2 interface show std_srvs/srv/SetBool 
# Result
bool data # e.g. for hardware enabling / disabling
---
bool success   # indicate successful run of triggered service
string message # informational, e.g. for error messages

```
### call
```bash title="service call"
ros2 service call /echo_service std_srvs/srv/SetBool "{data: True}"

# Result
requester: making request: std_srvs.srv.SetBool_Request(data=True)

response:
std_srvs.srv.SetBool_Response(success=True, message='success')

```

!!! Warning
    Space are mandatory between `data` and the `value`
    Example:  `{data: True}` from service call

---

# Reference
- [ros2 service Command Line Tool ](https://roboticsbackend.com/ros2-service-cmd-line-tool-debug-ros2-services/)
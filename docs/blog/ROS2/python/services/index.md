---
title: ROS2 Services
tags:
    - ros2
    - service
---

# Simple Server

```python title="simple_service.py" linenums="1" hl_lines="1"
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(SetBool, 'echo_service', self.echo_callback)

    def echo_callback(self, request: SetBool.Request, response: SetBool.Response):
        response.success = request.data
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

```bash
ros2 service call /echo_service std_srvs/srv/SetBool "{data: True}"
requester: making request: std_srvs.srv.SetBool_Request(data=True)

response:
std_srvs.srv.SetBool_Response(success=True, message='success')
```

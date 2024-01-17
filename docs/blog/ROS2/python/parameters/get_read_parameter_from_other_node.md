---
tags:
    - ros2
    - parameter
    - param
    - python
    - get
---

Get parameter from other Node using `node service`


```python
import rclpy
from rclpy.task import Future
from rclpy.node import Node
from rclpy.parameter import parameter_value_to_python
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.srv._get_parameters import GetParameters_Response

class MyNode(Node):
    def __init__(self):
        node_name="get_param_demo"
        super().__init__(node_name)
        self.params_names = ["persistent.some_int", 
                             "persistent.some_lists.some_integers",
                             "persistent.pi"]
        self.get_logger().info("Hello get_param")
        self.send_request()

    def send_request(self):
        req = GetParameters.Request()
        req.names = self.params_names
        future = self.client.call_async(req)
        future.add_done_callback(self.service_handler)

    def service_handler(self, future: Future):
        response: GetParameters_Response
        response = future.result()
        
        for name, item in zip(self.params_names, response.values):
            p = parameter_value_to_python(item)
            print(name, p)
       


def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
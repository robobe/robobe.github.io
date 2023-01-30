---
title: ROS2 update node parameter from client
tags:
    - ros2
    - param
---

Update parameter from other node, using `set_parameters` service

### demo
- set `my_int` param from `param basic` [node](parameter_update_node_event.md)
  

```bash title="terminal1: run node"
ros2 run pkg_python_tutorial param_control
```

```bash title="terminal2: list node params"
ros2 param list
/basic_param:
  my_int
  use_sim_time

```

#### Know your environment
```bash title="terminal2: list node services"
ros2 service list
/basic_param/describe_parameters
/basic_param/get_parameter_types
/basic_param/get_parameters
/basic_param/list_parameters
/basic_param/set_parameters
/basic_param/set_parameters_atomically
```

```bash title="terminal2: get service interface"
# get service interface
ros2 service type /basic_param/set_parameters
#
rcl_interfaces/srv/SetParameters
```

```python title="terminal2: show interface details" linenums="1" hl_lines="1 17"
Parameter[] parameters
	string name
	ParameterValue value
		uint8 type
		bool bool_value
		int64 integer_value
		float64 double_value
		string string_value
		byte[] byte_array_value
		bool[] bool_array_value
		int64[] integer_array_value
		float64[] double_array_value
		string[] string_array_value

---
# Indicates whether setting each parameter succeeded or not and why.
SetParametersResult[] results
	bool successful
	string reason
```



#### code

```python title="param_update_cllient.py" linenums="1" hl_lines="33 34 21"
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv._set_parameters import SetParameters_Response
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter_service import SetParameters

TOPIC = "/basic_param/set_parameters"

class MinimalClientAsync(Node):
    def __init__(self):
        super().__init__('update')
        self.cli = self.create_client(SetParameters, TOPIC)
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.send_request()
        
    def service_call_handler(self, future):
        response: SetParameters_Response
        result: SetParametersResult
        response = future.result()
        result = response.results[0]
        self.get_logger().info(f"{result.successful}")
        self.get_logger().info(result.reason)
        
        

    def send_request(self):
        self.req = SetParameters.Request()
        param_value = ParameterValue(integer_value=150)
        param_value.type = ParameterType.PARAMETER_INTEGER
        param = Parameter(name="my_int", value=param_value)
        params = [param]
        self.req.parameters = params
        self.future = self.cli.call_async(self.req)
        self.future.add_done_callback(self.service_call_handler)

def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    rclpy.spin(minimal_client)
    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
```

#### usage

!!! note "IntRange"
    `my_int` param has limit between 10-110
     

```bash title="terminal2"
# send value 150
ros2 run pkg_python_tutorial param_update
#
[INFO] [1674974296.640533953] [update]: False
[INFO] [1674974296.640736818] [update]: Parameter my_int out of range. Min: 10, Max: 110, value: 150

# send value 100
ros2 run pkg_python_tutorial param_update
[INFO] [1674974352.207990957] [update]: True
[INFO] [1674974352.208230578] [update]

```

!!! note "reason"
    `set_parameters` service response return 
    `SetParametersResult` object for each updated parameter,  
    the Object has two fields  
    - bool successful  
    - string reason  
    When update failed `reason` field contain the error msg
     
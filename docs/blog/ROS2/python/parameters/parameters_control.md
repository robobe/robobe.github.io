---
title: ROS2 Params more control
tags:
    - ros2
    - param
---

Base on  [previous](basic_example.md) let do more investigation on ROS2 params 

## Demo (global parameter event)
- Subscribe to `parameter_events` topic


```bash title="terminal1"
ros2 run pkg_python_tutorial param_basic
```

```bash title="terminal2"
ros2 topic list
#
/parameter_events
/rosout

# echo/subscribe topic
ros2 topic echo /parameter_events
```

```bash title="terminal3"
# change my_int param value
ros2 param set /basic my_int 20
```

```bash title="terminal2" linenums="1" hl_lines="4 7 11"
stamp:
  sec: 1674824296
  nanosec: 912765941
node: /basic
new_parameters: []
changed_parameters:
- name: my_int
  value:
    type: 2
    bool_value: false
    integer_value: 20
    double_value: 0.0
    string_value: ''
    byte_array_value: []
    bool_array_value: []
    integer_array_value: []
    double_array_value: []
    string_array_value: []
deleted_parameters: []
---

```

---

## Demo2 (control parameter from service)

Ros2 Node implement build in service to control Node parameters

```bash title="terminal1"
ros2 service list
#
/basic/describe_parameters
/basic/get_parameter_types
/basic/get_parameters
/basic/list_parameters
/basic/set_parameters
/basic/set_parameters_atomically
```

```bash title="terminal2"
ros2 service call /basic/list_parameters 
rcl_interfaces/srv/ListParameters
#
requester: making request: rcl_interfaces.srv.ListParameters_Request(prefixes=[], depth=0)

response:
rcl_interfaces.srv.ListParameters_Response(result=rcl_interfaces.msg.ListParametersResult(names=['use_sim_time', 'my_str', 'my_int'], prefixes=[]))

#

ros2 service call /basic/list_parameters rcl_interfaces/srv/ListParameters "{ prefixes: ["my"] }"
#
requester: making request: rcl_interfaces.srv.ListParameters_Request(prefixes=['my'], depth=0)

response:
rcl_interfaces.srv.ListParameters_Response(result=rcl_interfaces.msg.ListParametersResult(names=['my_str', 'my_int'], prefixes=[]))

```

!!! tip "call service tips"
    - The request data must be inside  `"`
    - Key, Value must have space between them `prefixes: ["my"]`
     

### get_parameters

```bash
ros2 service call /basic/get_parameters rcl_interfaces/srv/GetParameters "{ names: [my_int] }"
#
requester: making request: rcl_interfaces.srv.GetParameters_Request(names=['my_int'])

response:
rcl_interfaces.srv.GetParameters_Response(values=[rcl_interfaces.msg.ParameterValue(type=2, bool_value=False, integer_value=20, double_value=0.0, string_value='', byte_array_value=[], bool_array_value=[], integer_array_value=[], double_array_value=[], string_array_value=[])])

```
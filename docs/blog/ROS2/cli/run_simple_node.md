---
tags:
    - ros2
    - cli
    - namespace
    - remap
---

# Run ROS2 Node
Run minimal ROS2 node from:

- cli
- add namespace
- remap topic
- remap node name
- launch file
- cli control debug level
 
## Run from cli

```bash
ros2 run params_demos minimal
#
[INFO] [1680205241.556490384] [simple_param_node]: my_int: 3
[INFO] [1680205242.543293520] [simple_param_node]: my_int: 2
[INFO] [1680205243.543186445] [simple_param_node]: my_int: 1
[INFO] [1680205244.543174591] [simple_param_node]: close timer
```

```bash
ros2 node list
# 
/simple_param_node
```

## Add namespace
```bash title="terminal 1"
ros2 run params_demos minimal --ros-args --remap __ns:=/custom
```

```bash title="terminal 2"
ros2 node list           
#
/custom/simple_param_node

```

## Remap topic name
```bash title="terminal 1"
# without namespace
ros2 run params_demos minimal --ros-args --remap /simple_param_node/my_topic:=/new_topic

# with namespace
ros2 run params_demos minimal --ros-args --remap /custom/simple_param_node/my_topic:=/new_topic --remap __ns:=/custom
```

```bash title="terminal 2"
ros2 node list           
#
/custom/simple_param_node

ros2 topic list 
#
/new_topic
```

!!! tip "Topic absolute name"
    topic name start with forward slash are **absolute**    
    when you rename topic and the remap topic name are not absolute.   namespace add to the topic name as prefix

    ```bash
    ros2 run params_demos minimal --ros-args --remap /custom/simple_param_node/my_topic:=new_topic --remap __ns:=/custom

    ros2 topic list 
    #
    /custom_ns/new_topic
    ```

    ```bash
    ros2 run params_demos minimal --ros-args --remap /custom/simple_param_node/my_topic:=/new_topic --remap __ns:=/custom

    ros2 topic list 
    #
    /new_topic
    ```

!!! tip "Add fqn to topic name"
    [Topic and Service name mapping to DDS](https://design.ros2.org/articles/topic_and_service_names.html#fully-qualified-names)

    ```python
    fqn = self.get_fully_qualified_name()
    self.get_logger().info(f"node name: {node_name}")
    self.pub = self.create_publisher(Int32, fqn + TOPIC, 10)
    ```

## Remap node name

```bash title="Terminal 1"
ros2 run params_demos minimal --ros-args --remap __node:=node_new_name
```

```bash title="Terminal 2"
ros2 node list
#
/node_new_name

ros2 topic list
#
/node_new_name/my_topic
/parameter_events
/rosout
```
     
### Remap node name with namespace

```bash title="Terminal 1"
run params_demos minimal --ros-args --remap __node:=node_new_name --remap __ns:=/custom
```

```bash title="Terminal 2"
ros2 node list 
#
/custom/node_new_name

ros2 topic list
#
/custom/node_new_name/my_topic
/parameter_events
/rosout
```

!!! note node get name API
    
    ```python
    name = self.get_name()
    ns = self.get_namespace()
    fqn = self.get_fully_qualified_name()
    self.get_logger().info(f"node name: {name}")
    self.get_logger().info(f"ns: {ns}")
    self.get_logger().info(f"fqn: {fqn}")
    ```

    ```bash
    ros2 run params_demos minimal --ros-args --remap __node:=node_new_name --remap __ns:=/custom
    [INFO] [1680233355.573926310] [custom.node_new_name]: node name: node_new_name
    [INFO] [1680233355.574155685] [custom.node_new_name]: ns: /custom
    [INFO] [1680233355.574342657] [custom.node_new_name]: fqn: /custom/node_new_name

    ``` 

## launch

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    sim_node =  Node(
            package='params_demos',
            namespace='custom',
            executable='minimal',
            name='simple_node'
        )


    ld.add_action(sim_node)
    return ld
```

```bash
ros2 node list
/custom/simple_node
#
ros2 topic list
/custom/simple_node/my_topic
```

### remap topics

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    sim_node =  Node(
            package='params_demos',
            namespace='custom',
            executable='minimal',
            name='simple_node',
            remappings=[
                ("/custom/simple_node/my_topic", "/new_topic_name")
            ]
        )


    ld.add_action(sim_node)
    return ld
```

```bash
ros2 node list
/custom/simple_node
#
ros2 topic list
/new_topic_name

```

---

## cli and launch logging level control
[more ros2 logging info](https://robobe.github.io/blog/ROS2/tutorials/logging/?h=log#logging-control-demo)

### cli
loging level:

- debug
- info
- warn
- error


```bash title="terminal"
ros2 run params_demos minimal --ros-args --log-level debug
```

### launch

```python title="" linenums="1" hl_lines="15"
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    sim_node =  Node(
            package='params_demos',
            namespace='custom',
            executable='minimal',
            name='simple_node',
            remappings=[
                ("/custom/simple_node/my_topic", "/new_topic_name")
            ],
            arguments=['--ros-args', '--log-level', 'warn']
        )


    ld.add_action(sim_node)
    return ld
```

---

# Reference
- [ROS Command Line Arguments](https://design.ros2.org/articles/ros_command_line_arguments.html)
- [Topic and Service name mapping to DDS](https://design.ros2.org/articles/topic_and_service_names.html#fully-qualified-names)
---
title: Part2a - Launch file part II
description: Basic ROS2 launch file , control node name and namespace
date: "2022-04-27"
banner: ../ros2.png
tags:
    - ros2
    - launch
    - 101
---

## launch source code
```python title="hello.launch.py" linenums="1" hl_lines="9 10 16 17"
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    talker_node = Node(
        package="basic",
        namespace="my_ns",
        name="my_pub",
        executable="simple_pub",
    )

    listener_node = Node(
        package="basic",
        namespace="my_ns",
        name="my_sub",
        executable="simple_sub"
    )

    ld.add_action(talker_node)
    ld.add_action(listener_node)

    return ld
```

## Usage
- launch
- node list
- topic list
- rqt_graph

### launch
- log output note ns.node_name

```bash
ros2 launch basic hello.launch.py 
...
[simple_pub-1] [INFO] [1651090792.927189288] [my_ns.my_pub]: Publishing: "pub simple: 1"
[simple_sub-2] [INFO] [1651090792.927732100] [my_ns.my_sub]: I heard: pub simple: 1
```

### node list
```bash
ros2 node list
# Result
/my_ns/my_pub
/my_ns/my_sub
```

### topic list
```bash
ros2 topic list
/my_ns/minimal
/parameter_events
/rosout
```

!!! Node
    - `/rosout`: ??
    - `/parameter_event`: ??

### rqt_graph

![](images/rqt_graph.png)


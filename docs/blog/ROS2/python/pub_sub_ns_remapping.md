---
title: Minimal Pub/Sub with namespace and remapping topics
tags:
    - python
    - ros2
---

# LAB
- Create minimal Pub/Sub with different topic
- remapping topic from command line
- remapping from launch
- add namespace
- change node name

---

## Minimal nodes

```python title="minimal_pub.py"
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

TOPIC = "simple"
PERIOD = 1

class MyNode(Node):
    def __init__(self):
        node_name="minimal_pub"
        super().__init__(node_name)
        self.__pub = self.create_publisher(String, TOPIC, 10)
        self.__timer = self.create_timer(PERIOD, self.__timer_handler)
        self.__timer
        self.__counter = 0
        self.get_logger().info("run simple pub")

    def __timer_handler(self):
        self.__counter += 1
        msg = String(data="pub counter: {}".format(self.__counter))
        self.__pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

```python title="minimal_sub.py"
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

TOPIC = "simple1"

class MyNode(Node):
    def __init__(self):
        node_name="minimal_sub"
        super().__init__(node_name)
        self.__sub = self.create_subscription(String, TOPIC, self.__sub_handler, 10)
        self.__sub
        self.get_logger().info("start minimal sub")

    def __sub_handler(self, msg: String):
        self.get_logger().info(msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

```python title="setup.py"
# Add entry points
entry_points={
        'console_scripts': [
            "minimal_pub=py_tutorial_pkg.minimal_pub:main",
            "minimal_sub=py_tutorial_pkg.minimal_sub:main"
        ],
    }
```

## first run

```bash title="terminal1"
ros2 run py_tutorial_pkg minimal_pub
```

```bash title="terminal2"
ros2 run py_tutorial_pkg minimal_sub
```

```bash title="terminal3"
# nodes
ros2 node list
/minimal_pub
/minimal_sub

# topics
ros2 topic list
/parameter_events
/rosout
/simple
/simple1

# info topic /simple
ros2 topic info /simple
Type: std_msgs/msg/String
Publisher count: 1
Subscription count: 0

# info topic /simple1
ros2 topic info /simple1
Type: std_msgs/msg/String
Publisher count: 0
Subscription count: 1
```

## usage remapping sub topic

```bash title="terminal2"
ros2 run py_tutorial_pkg minimal_sub --ros-args -r simple1:=simple
```

```bash title="terminal3"
ros2 topic list
/parameter_events
/rosout
/simple
```

## launch with remapping

```python title="run_minimal_1.launch.py"
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    pub_node =  Node(
            package='py_tutorial_pkg',
            executable='minimal_pub'
        )

    sub_node =  Node(
            package='py_tutorial_pkg',
            executable='minimal_sub',
            remappings=[
                ('/simple1', '/simple'),
            ]
        )

    ld.add_action(pub_node)
    ld.add_action(sub_node)
    return ld
```

```bash title="terminal1"
ros2 launch py_tutorial_pkg run_minimal1.launch.py
```

```bash title="terminal2"
# nodes
ros2 node list
/minimal_pub
/minimal_sub

# topics
ros2 topic list
/parameter_events
/rosout
/simple
```

---

## Add namespace

```bash title="terminal1"
ros2 run py_tutorial_pkg minimal_pub --ros-args -r __ns:=/demo
```

```bash title="terminal2"
ros2 run py_tutorial_pkg minimal_sub --ros-args -r __ns:=/other_demo
```

```bash title="terminal3"
# nodes
ros2 node list
/demo/minimal_pub
/other_demo/minimal_sub

# topics
ros2 topic list
/demo/simple
/other_demo/simple1

```

### remap pub this time

```bash title="terminal1"
ros2 run py_tutorial_pkg minimal_pub --ros-args -r __ns:=/demo -r /demo/simple:=/other_demo/simple1

```

```bash title="terminal3"
# nodes
ros2 topic list
/demo/other_demo/simple1
/other_demo/simple1

# topics
ros2 topic list
/other_demo/simple1
```

### with launch file

- Add namespace
- Remap topic with full namespace


```python title="run_minimal_2.launch.py"
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    pub_node =  Node(
            package='py_tutorial_pkg',
            executable='minimal_pub',
            namespace="/demo",
            remappings=[
                ('/demo/simple', '/other_demo/simple1'),
            ]
        )

    sub_node =  Node(
            package='py_tutorial_pkg',
            executable='minimal_sub',
            namespace="/other_demo"
            
        )

    ld.add_action(pub_node)
    ld.add_action(sub_node)
    return ld
```
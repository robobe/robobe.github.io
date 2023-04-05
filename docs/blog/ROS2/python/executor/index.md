---
tags:
    - ros2
    - python
    - executor
    - MultiThreadedExecutor
    - SingleThreadedExecutor
---
# ROS2 rclpy executor

By default, rclpy offers two different executors for the user to choose from:

- SingleThreadedExecutor (default)
- MultiThreadedExecutor

**SingleThreadedExecutor**:  executes callbacks in a single thread, one at a time, and thus the previous callback must always finish before a new one can begin execution.

**MultiThreadedExecutor**: executing several callbacks simultaneously.

## Callback groups
ROS 2 allows organizing the callbacks of a node in groups.

- MutuallyExclusiveCallbackGroup
- ReentrantCallbackGroup

**MutuallyExclusiveCallbackGroup**: Callbacks of this group must not be executed in parallel.

**ReentrantCallbackGroup**: Callbacks of this group may be executed in parallel.

callback examples
- subscription callback
- timer callback
- service callback (request on server)
- action server and client callback
- Future done callback

## demo

Node with timer that handle long time work,
the demo assign timer callback to each type of group with MultiThreadedExecutor

### ReentrantCallbackGroup
```python title="ReentrantCallbackGroup"
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import time
import threading

WORKER_THREAD = 10
WORKER_10HZ = 1/10
PAYLOAD_TIME = 1/2

class MyNode(Node):
    def __init__(self):
        node_name="executer_demo"
        super().__init__(node_name)
        g = ReentrantCallbackGroup()
        self.create_timer(WORKER_10HZ, self.__timer_handler, callback_group=g)
        self.get_logger().info("Hello ROS2")

    def __timer_handler(self):
        self.get_logger().info(f"work: {threading.current_thread().name}")
        time.sleep(PAYLOAD_TIME)

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    try:
        exec = MultiThreadedExecutor(WORKER_THREAD)
        exec.add_node(node)
        exec.spin()
    except KeyboardInterrupt:
        print("User exit")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
```

```bash
[INFO] [1680619887.064644971] [keyboard]: work: ThreadPoolExecutor-0_2
[INFO] [1680619887.163538444] [keyboard]: work: ThreadPoolExecutor-0_3
[INFO] [1680619887.264196196] [keyboard]: work: ThreadPoolExecutor-0_4
[INFO] [1680619887.364816744] [keyboard]: work: ThreadPoolExecutor-0_5
[INFO] [1680619887.464378506] [keyboard]: work: ThreadPoolExecutor-0_0
[INFO] [1680619887.564040306] [keyboard]: work: ThreadPoolExecutor-0_6
[INFO] [1680619887.665020484] [keyboard]: work: ThreadPoolExecutor-0_1
[INFO] [1680619887.764469847] [keyboard]: work: ThreadPoolExecutor-0_2
[INFO] [1680619887.864734746] [keyboard]: work: ThreadPoolExecutor-0_3
[INFO] [1680619887.964096619] [keyboard]: work: ThreadPoolExecutor-0_4


```

The executer success to run payload in time interval 10hz

### MutuallyExclusiveCallbackGroup
```python title="MutuallyExclusiveCallbackGroup"
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import time
import threading

WORKER_THREAD = 10
WORKER_10HZ = 1/10
PAYLOAD_TIME = 1/2

class MyNode(Node):
    def __init__(self):
        node_name="executer_demo"
        super().__init__(node_name)
        g = MutuallyExclusiveCallbackGroup()
        self.create_timer(WORKER_10HZ, self.__timer_handler, callback_group=g)
        self.get_logger().info("Hello ROS2")

    def __timer_handler(self):
        self.get_logger().info(f"work: {threading.current_thread().name}")
        time.sleep(PAYLOAD_TIME)

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    try:
        exec = MultiThreadedExecutor(WORKER_THREAD)
        exec.add_node(node)
        exec.spin()
    except KeyboardInterrupt:
        print("User exit")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
```

```bash
[INFO] [1680619992.063696171] [executer_demo]: work: ThreadPoolExecutor-0_0
[INFO] [1680619992.565431058] [executer_demo]: work: ThreadPoolExecutor-0_1
[INFO] [1680619993.068422713] [executer_demo]: work: ThreadPoolExecutor-0_0
[INFO] [1680619993.571697147] [executer_demo]: work: ThreadPoolExecutor-0_1
[INFO] [1680619994.073350571] [executer_demo]: work: ThreadPoolExecutor-0_0
[INFO] [1680619994.576600477] [executer_demo]: work: ThreadPoolExecutor-0_1
```

The timer success to run the payload only in serial one ofter anther
---

# Reference
- [Deadlocks in rclpy and how to prevent them with use of callback groups](https://karelics.fi/deadlocks-in-rclpy/)
- [Executors](https://docs.ros.org/en/humble/Concepts/About-Executors.html)
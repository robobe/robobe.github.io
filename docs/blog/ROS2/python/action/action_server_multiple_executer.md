---
title: Create Action server with multithread executer
tags:
    - ros2
    - python
    - action
---

## Demo

```python title="action server with multithread" linenums="1" hl_lines="50"
import rclpy
import threading
import time
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.action.server import ServerGoalHandle

from action_tutorial_interfaces.action import MyAction

TOPIC = "my_action_demo"
PERIOD = 1

class MyActionServer(Node):
    def __init__(self):
        super().__init__('my_action_server')
        self._action_server = ActionServer(
            self,
            MyAction,
            TOPIC,
            self.execute_callback)
        self.__timer = self.create_timer(PERIOD, self.__timer_handler)
        self.__timer
        self.get_logger().info('Start my_action_server version')

    def __timer_handler(self):
        self.get_logger().info("timer thread ->: {}".format(threading.current_thread().ident))
        

    def execute_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("action thread ->: {}".format(threading.current_thread().ident))
        feedback_msg = MyAction.Feedback()
        for i in range(goal_handle.request.count):
            time.sleep(1)
            self.get_logger().info('current: {}'.format(i))
            feedback_msg.current = i
            goal_handle.publish_feedback(feedback_msg)

        self.get_logger().info('Action ended')
        goal_handle.succeed()

        result = MyAction.Result()
        result.total = i
        return result


def main(args=None):
    rclpy.init(args=args)
    action_server = MyActionServer()
    exec = MultiThreadedExecutor()
    exec.add_node(action_server)
    exec.spin()
    rclpy.spin(action_server)


if __name__ == '__main__':
    main()
```

### Run

```bash title="run action"
ros2 action send_goal -f /my_action_demo action_tutorial_interfaces/action/MyAction "{count: 3}"
```

```bash title="server output" linenums="1" hl_lines="1 3 5 8"
[INFO] [1670248402.659192149] [my_action_server]: timer thread ->: 140488007521856
[INFO] [1670248403.657669396] [my_action_server]: timer thread ->: 140487998469696
[INFO] [1670248404.005261426] [my_action_server]: action thread ->: 140487998469696
[INFO] [1670248404.658963720] [my_action_server]: timer thread ->: 140488007521856
[INFO] [1670248405.006998918] [my_action_server]: current: 0
[INFO] [1670248405.658867664] [my_action_server]: timer thread ->: 140487990076992
[INFO] [1670248406.009163793] [my_action_server]: current: 1
[INFO] [1670248406.658945210] [my_action_server]: timer thread ->: 140488015914560
[INFO] [1670248407.011518224] [my_action_server]: current: 2
[INFO] [1670248407.012507114] [my_action_server]: Action ended
[INFO] [1670248407.658774598] [my_action_server]: timer thread ->: 140488015914560

```

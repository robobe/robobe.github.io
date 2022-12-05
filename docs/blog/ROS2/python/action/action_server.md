---
title: Create Action server
tags:
    - ros2
    - python
    - action
---

!!! tip "VSCode action intellisense"
    Add path to search to `python.analysis.extraPaths` list
    in settings.json

    ```json
    {
    "python.analysis.extraPaths": [
        "/home/user/ros2_ws/install/action_tutorial_interfaces/local/lib/python3.10/dist-packages"
        ]
    }
    ```
     
## demo

```python title="action_tutorial/my_server.py"
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.action.server import ServerGoalHandle
from action_tutorial_interfaces.action import MyAction

TOPIC = "my_action_demo"

class MyActionServer(Node):
    def __init__(self):
        super().__init__('my_action_server')
        self._action_server = ActionServer(
            self,
            MyAction,
            TOPIC,
            self.execute_callback)
        self.get_logger().info('Start my_action_server version')

    def execute_callback(self, goal_handle: ServerGoalHandle):
        feedback_msg = MyAction.Feedback()
        for i in range(goal_handle.request.count):
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
    rclpy.spin(action_server)

if __name__ == '__main__':
    main()
```

### Test




```bash title="send_goal" linenums="1" hl_lines="3 4 8 23 26"
ros2 action send_goal -f /my_action_demo action_tutorial_interfaces/action/MyAction "{count: 5}"
Waiting for an action server to become available...
Sending goal:
     count: 5

Goal accepted with ID: c1129edf0f894012921b849b7948703a

Feedback:
    current: 0

Feedback:
    current: 1

Feedback:
    current: 2

Feedback:
    current: 3

Feedback:
    current: 4

Result:
    total: 4

Goal finished with status: SUCCEEDED

```
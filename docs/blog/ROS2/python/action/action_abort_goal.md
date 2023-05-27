---
tags:
    - ros2
    - rclpy
    - action
    - goal abort
---

- Each client goal request abort the previous one
- When new accepted goal received, the server check for previous goal (36) if exists it's abort it.
- The execute callback check if the goal_handle still active , is aborted the goal not active and the loop ended


## Server

```python title="server" linenums="1" hl_lines="28 33-42 52-54"
import threading

import rclpy
from custom_interfaces.action._counter import Counter_Goal, Counter_Result
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from custom_interfaces.action import Counter  # pylint: disable=E0401

TOPIC = "my_action_demo"


class MyNode(Node):
    def __init__(self):
        node_name = "action_server"
        super().__init__(node_name)
        self._goal_handle = None
        self._goal_lock = threading.Lock()
        self._action_server = ActionServer(
            self,
            Counter,
            TOPIC,
            self.execute_callback,
            handle_accepted_callback=self.__accepted_handler,
            callback_group=ReentrantCallbackGroup(),
        )
        self.get_logger().info("Hello ROS2")

    def __accepted_handler(self, goal_handle: ServerGoalHandle) -> None:
        with self._goal_lock:
            # This server only allows one goal at a time
            if self._goal_handle is not None and self._goal_handle.is_active:
                self.get_logger().info("Aborting previous goal")
                # Abort the existing goal
                self._goal_handle.abort()
            self._goal_handle = goal_handle

        goal_handle.execute()

    def execute_callback(self, goal_handle: ServerGoalHandle) -> Counter_Result:
        goal: Counter_Goal
        feedback_msg = Counter.Feedback()
        goal = goal_handle.request
        self.get_logger().info(
            f"execute method thread: {threading.current_thread().name}"
        )
        for i in range(goal.count):
            if not goal_handle.is_active:
                self.get_logger().info("Goal aborted")
                return Counter.Result()

            self.get_logger().info(f"publish feedback: {i}")
            feedback_msg.current = i
            goal_handle.publish_feedback(feedback_msg)
            self.get_clock().sleep_for(rel_time=Duration(seconds=1))

        self.get_logger().info("Action ended")
        goal_handle.succeed()

        result = Counter.Result()
        result.total = i
        return result


def main():
    rclpy.init()
    executor = MultiThreadedExecutor()
    node = MyNode()
    node.get_logger().info(f"main method thread: {threading.current_thread().name}")
    try:
        rclpy.spin(node, executor)
    except KeyboardInterrupt:
        print("User exit")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

```

## client

```python title="client" linenums="1" hl_lines="32"
import rclpy
from rclpy.node import Node
from rclpy.action.client import ActionClient, ClientGoalHandle
from custom_interfaces.action import Counter

ACTION_NAME = "my_action_demo"

class SimpleClient(Node):
    def __init__(self):
        node_name="action_client_demo"
        super().__init__(node_name)
        self.action_client = ActionClient(self, Counter, ACTION_NAME)
        self.action_client.wait_for_server(1.0)
        self.get_logger().info("Hello ROS2")
        self.__timer = None
        self.__goal_handler = None
        self.goal_request(request_counter=10)

    def goal_request(self, request_counter: int) -> None:
        goal = Counter.Goal()
        goal.count = request_counter
        future = self.action_client.send_goal_async(goal)
        future.add_done_callback(self.goal_handler)

    def goal_handler(self, future):
        self.__goal_handler: ClientGoalHandle
        self.__goal_handler = future.result()
        if self.__goal_handler.accepted:
            self.__timer = self.create_timer(2.0, self.__handler_timer)

    def __handler_timer(self):
        # Send again
        self.__timer.cancel()
        goal = Counter.Goal()
        goal.count = 5
        self.action_client.send_goal_async(goal)


        
def main(args=None):
    rclpy.init(args=args)
    node = SimpleClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("User exit")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
```
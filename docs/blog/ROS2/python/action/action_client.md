

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import threading

from action_tutorial_interfaces.action import MyAction

TOPIC = "my_action_demo"

class MyActionClient(Node):

    def __init__(self):
        super().__init__('action_client')
        self._action_client = ActionClient(self,
            MyAction,
            TOPIC)

    def send_goal(self, count):
        goal_msg = MyAction.Goal()
        goal_msg.count = count
        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        self.get_logger().info("feedback thread {}".format(threading.current_thread().ident))

    def goal_response_callback(self, future):
        self.get_logger().info("response thread {}".format(threading.current_thread().ident))
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)


    def get_result_callback(self, future):
        self.get_logger().info("result thread {}".format(threading.current_thread().ident))
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.total))
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    action_client = MyActionClient()
    action_client.send_goal(10)
    rclpy.spin(action_client)
    

if __name__ == '__main__':
    main()
```
---
tags:
    - ros2
    - python
    - action server
---
# Action server examples and API

[API (foxy)](https://docs.ros2.org/foxy/api/rclpy/api/actions.html#module-rclpy.action.server)

!!! warning missing humble rclpy api documentation
    humble document can found in [humble api](http://docs.ros.org/en/humble/p/) but there are no reference to **rclpy**


**goal_callback**: callback function to handle new client goal request, return `GoalResponse.REJECT` or `GoalResponse.ACCEPT`

```python title="" linenums="1" hl_lines="5"
def __init__(self):
        node_name = "action_server"
        super().__init__(node_name)
        self._action_server = ActionServer(
            self,
            Counter,
            TOPIC,
            self.execute_callback,
            goal_callback=self.__handle_goal_callback,
        )
        self.get_logger().info("Hello ROS2")

    def __handle_goal_callback(self, goal: Counter_Goal) -> GoalResponse:
        if goal.count < 5:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT
```

```
ros2 service type /my_action_demo/_action/cancel_goal action_msgs/srv/CancelGoal
```
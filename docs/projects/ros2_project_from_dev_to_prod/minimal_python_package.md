---
title: ROS2 python project deploy minimal package
tags:
    - ros2
    - projects
    - deploy
---

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from std_msgs.msg import Int32

TOPIC = "my_topic"
PERIOD = 1.0


class MyNode(Node):
    def __init__(self) -> None:
        node_name = "minimal_pub"
        super().__init__(node_name)
        self.__pub = self.create_publisher(
            Int32, TOPIC, qos_profile=qos_profile_system_default
        )
        self.__timer = self.create_timer(PERIOD, self.__timer_handler)
        self.__counter = 0
        self.get_logger().info("start minimal pub")

    def __timer_handler(self) -> int:
        msg = Int32()
        msg.data = self.__counter
        self.__pub.publish(msg)
        self.__counter += 1
        self.get_logger().info(f"publish message count: {self.__counter}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("User exit")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

```
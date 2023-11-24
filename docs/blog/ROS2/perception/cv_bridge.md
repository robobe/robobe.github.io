---
tags:
    - ros2
    - opencv
    - perception
    - bridge
---

# ROS2 CV2 bridge

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

FRONT_CAMERA_TOPIC = "/front_camera/image_raw"

class MyNode(Node):
    def __init__(self):
        node_name="cv_bridge_demo"
        super().__init__(node_name)
        self.get_logger().info("Hello CV and ROS2")
        self.img_sub = self.create_subscription(
            Image,
            FRONT_CAMERA_TOPIC,
            self.image_handler,
            qos_profile=qos_profile_sensor_data)
        self.cv_br = CvBridge()

    def image_handler(self, msg: Image):
        frame = self.cv_br.imgmsg_to_cv2(msg)
        cv2.imshow("brige", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

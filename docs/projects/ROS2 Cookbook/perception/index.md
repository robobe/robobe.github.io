---
title: ROS2 perception start here
tags:
    - cv_bridge
---

```bash title="install"
sudo apt -y install ros-foxy-vision-opencv
sudo apt -y install ros-foxy-v4l2-camera
# opencv from source or pip
# pip3 install opencv-python
```

```xml title="urdf" linenums="1" hl_lines="1"
<gazebo reference="camera_link">
<sensor type="camera" name="camera1">
    <update_rate>15.0</update_rate>
    <camera name="head">
    <horizontal_fov>1.3962634</horizontal_fov>
    <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
    </image>
    <clip>
        <near>0.02</near>
        <far>300</far>
    </clip>
    <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.007</stddev>
    </noise>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
    <alwaysOn>true</alwaysOn>
    <updateRate>0.0</updateRate>
    <cameraName>rrbot/camera1</cameraName>
    <imageTopicName>image_raw</imageTopicName>
    <cameraInfoTopicName>camera_info</cameraInfoTopicName>
    <frameName>camera_link_optical</frameName>
    <hackBaseline>0.0</hackBaseline>
    <distortionK1>0.0</distortionK1>
    <distortionK2>0.0</distortionK2>
    <distortionK3>0.0</distortionK3>
    <distortionT1>0.0</distortionT1>
    <distortionT2>0.0</distortionT2>
    <CxPrime>0</CxPrime>
    <Cx>0.0</Cx>
    <Cy>0.0</Cy>
    <focalLength>0.0</focalLength>
    </plugin>
</sensor>
</gazebo>
```

## CvBridge

```python
import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

CAMERA_TOPIC = "/camera1/image_raw"


class ImageViewer(Node):
    def __init__(self):
        super().__init__("viewer")
        self.image_sub = self.create_subscription(Image, CAMERA_TOPIC, self.callback, 10)
        self.bridge = CvBridge()

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        cv2.imshow("image", cv_image)
        cv2.waitKey(3)

def main(args=None):
    try:
        rclpy.init()
        image_converter = ImageViewer()
        rclpy.spin(image_converter)
        image_converter.destroy_node
        rclpy.shutdown()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()
if __name__ == '__main__':   
    main()
```
---
tags:
    - rosbags
    - tools
    - ros1
    - ros2
---

# Rosbags

Rosbags is the pure python library for everything rosbag. It contains:

- highlevel easy-to-use interfaces,
- rosbag2 reader and writer,
- rosbag1 reader and writer,
- extensible type system with serializers and deserializers
- efficient converter between rosbag1 and rosbag2,
- and more.
[continue reading](https://gitlab.com/ternaris/rosbags)


### install
```bash
pip install rosbags

```

### usage
Convert ros1 bag file to folder contain sqlite3 db file and yaml file with bag metadata
 
```bash title="convert ros1 to ros2 bag"
./.local/bin/rosbags-convert <bah file>.bag
```


#### Demo
TODO: finish the example

```python
# https://github.com/vanbreugel-lab/optic_flow_example/tree/master
# https://ternaris.gitlab.io/rosbags/topics/rosbag1.html
from pathlib import Path
import cv2
import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore
from cv_bridge import CvBridge
from rosbags.rosbag2 import Writer
import rclpy
from sensor_msgs.msg import Image

rclpy.init()
# from rosbags.typesys.stores.ros2_humble import (
#     builtin_interfaces__msg__Time as Time,
#     sensor_msgs__msg__Image as Image,
#     sensor_msgs__msg__CompressedImage as CompressedImage,
#     std_msgs__msg__Header as Header
# )
bridge = CvBridge()
typestore2 = get_typestore(Stores.ROS2_HUMBLE)
# Image = typestore2.types['sensor_msgs/msg/Image']
node = rclpy.create_node('test')
pub = node.create_publisher(Image, '/camera', 10)
bagpath = Path('/home/user/tmp/optic_flow_lab.bag')
bagpath2 = Path('/home/user/tmp/optic_flow_lab2.bag')
typestore = get_typestore(Stores.ROS1_NOETIC)

# writer = Writer(bagpath2)
# writer.open()
TOPIC = "/camera"
# conn = writer.add_connection(TOPIC, Image.__msgtype__, typestore=typestore2)

timestamp = 0
with AnyReader([bagpath], default_typestore=typestore) as reader:
    for connection in reader.connections:
        print(connection.topic, connection.msgtype)

    for connection, timestamp, rawdata in reader.messages():
        msg = typestore.deserialize_ros1(rawdata, connection.msgtype)
        print(msg.header.frame_id)
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        msg = bridge.cv2_to_imgmsg(cv_image, "bgr8")
        pub.publish(msg)
        # msg = Image(
        #         Header(
        #             stamp=Time(sec=int(timestamp // 10**9), nanosec=int(timestamp % 10**9)),
        #             frame_id="map",
        #         ),
        #         encoding='rgb8', data=cv_image.tobytes() ,
        #         width=cv_image.shape[1],
        #         height=cv_image.shape[0],
        #         step=cv_image.shape[1] * 3,
        #         is_bigendian=False
        #     )

        # writer.write(conn, timestamp, typestore2.serialize_cdr(msg, msg.__msgtype__))
        # timestamp+=1
        cv2.imshow('frame', cv_image)
        cv2.waitKey(10)
```
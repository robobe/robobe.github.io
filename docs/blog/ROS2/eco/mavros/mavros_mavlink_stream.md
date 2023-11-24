---
tags:
    - mavros
    - ros2
    - ardupilot
    - mavlink
---

# Using mavros to listing to mavlink input stream

```
ros2 launch mavros apm.launch fcu_url:=udp://:14550@
```

```bash
ros2 topic list 
#
...
/uas1/mavlink_sink
/uas1/mavlink_source
```

```bash
ros2 topic info /uas1/mavlink_sink 
Type: mavros_msgs/msg/Mavlink
Publisher count: 1
Subscription count: 1
```

```bash title="mavros mavlink msg"
ros2 interface show mavros_msgs/msg/Mavlink
```

- msgid
- payload

```bash title="echo mavlink_sink"
ros2 topic echo /uas1/mavlink_sink
header:
  stamp:
    sec: 1699995913
    nanosec: 351881791
  frame_id: mavros
framing_status: 1
magic: 253
len: 9
incompat_flags: 0
compat_flags: 0
seq: 219
sysid: 1
compid: 191
msgid: 0
checksum: 9229
payload64:
- 342282445082591232
- 2362627
signature: []

```

---

## pymavlink
parse incoming message using pymavlink

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from mavros import mavlink
from mavros_msgs.msg import Mavlink
from pymavlink.dialects.v20 import ardupilotmega

class MyNode(Node):
    def __init__(self):
        node_name="mavros_pymavlink"
        super().__init__(node_name)
        self.get_logger().info("hello mavros and pymavlink")
        self.create_subscription(Mavlink, "/uas1/mavlink_sink", self.mavlink_incoming_handler, qos_profile=qos_profile_sensor_data)
        self.mavlink_dialect = ardupilotmega.MAVLink(None)
        
    def mavlink_incoming_handler(self, msg: Mavlink):
        raw = mavlink.convert_to_bytes(msg)
        mav_msg = self.mavlink_dialect.decode(raw)
        print(mav_msg)



def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
---

## Reference
- [Working with MAVlink messages in ROS without Mavros](https://james1345.medium.com/working-with-mavlink-messages-in-ros-without-mavros-88a055973fdf)
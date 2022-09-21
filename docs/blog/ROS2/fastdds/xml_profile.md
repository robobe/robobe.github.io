---
title: ROS2 fastdds xml profile
tags:
    - profile
    - qos
    - ros2
---

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy import qos

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__("minimal_publisher")
        topic_name = "/minimal"
        self.publisher_ = self.create_publisher(String, topic_name, qos_profile=qos.qos_profile_system_default)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        self.i += 1
        msg = String()
        msg.data = f"Hello ROS2 {self.i}"
        self.publisher_.publish(msg)
        self.get_logger().info(f"send: {msg.data} ")


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    # Spin the node so the callback function is called.
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

```

```xml title="profiles.xml"
<?xml version="1.0" encoding="UTF-8" ?>
<!-- 
    export RMW_FASTRTPS_USE_QOS_FROM_XML=1
    export FASTRTPS_DEFAULT_PROFILES_FILE=/home/user/dev_ws/src/basic/profiles.xml
-->
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">

    <!-- default publisher profile -->
    <publisher profile_name="default_publisher" is_default_profile="true">
        <historyMemoryPolicy>DYNAMIC</historyMemoryPolicy>
        <qos>
            <reliability>
                <kind>RELIABLE</kind>
            </reliability>
        </qos>
    </publisher>

    <!-- publisher profile for  minimal topic-->
    <publisher profile_name="/minimal">
        <historyMemoryPolicy>DYNAMIC</historyMemoryPolicy>
        <qos>
            <publishMode>
                <kind>ASYNCHRONOUS</kind>
            </publishMode>
            <reliability>
                <kind>BEST_EFFORT</kind>
            </reliability>
        </qos>
    </publisher>
 </profiles>
```

```bash
# environment
export RMW_FASTRTPS_USE_QOS_FROM_XML=1
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/user/dev_ws/src/basic/profiles.xml
# run node
ros2 run basic simple_pub
# result
2022-09-21 05:50:26.183 [RTPS_READER_HISTORY Error] Change payload size of '72' bytes is larger than the history payload size of '35' bytes and cannot be resized. -> Function can_change_be_added_nts
2022-09-21 05:50:26.184 [RTPS_READER_HISTORY Error] Change payload size of '96' bytes is larger than the history payload size of '35' bytes and cannot be resized. -> Function can_change_be_added_nts
2022-09-21 05:50:26.203 [RTPS_READER_HISTORY Error] Change payload size of '120' bytes is larger than the history payload size of '35' bytes and cannot be resized. -> Function can_change_be_added_nts
2022-09-21 05:50:26.204 [RTPS_READER_HISTORY Error] Change payload size of '168' bytes is larger than the history payload size of '35' bytes and cannot be resized. -> Function can_change_be_added_nts
2022-09-21 05:50:26.204 [RTPS_READER_HISTORY Error] Change payload size of '216' bytes is larger than the history payload size of '35' bytes and cannot be resized. -> Function can_change_be_added_nts
2022-09-21 05:50:26.204 [RTPS_READER_HISTORY Error] Change payload size of '264' bytes is larger than the history payload size of '35' bytes and cannot be resized. -> Function can_change_be_added_nts
2022-09-21 05:50:26.204 [RTPS_READER_HISTORY Error] Change payload size of '312' bytes is larger than the history payload size of '35' bytes and cannot be resized. -> Function can_change_be_added_nts
2022-09-21 05:50:26.204 [RTPS_READER_HISTORY Error] Change payload size of '360' bytes is larger than the history payload size of '35' bytes and cannot be resized. -> Function can_change_be_added_nts
2022-09-21 05:50:26.205 [RTPS_READER_HISTORY Error] Change payload size of '408' bytes is larger than the history payload size of '35' bytes and cannot be resized. -> Function can_change_be_added_nts
2022-09-21 05:50:26.205 [RTPS_READER_HISTORY Error] Change payload size of '432' bytes is larger than the history payload size of '35' bytes and cannot be resized. -> Function can_change_be_added_nts
2022-09-21 05:50:26.335 [RTPS_READER_HISTORY Error] Change payload size of '152' bytes is larger than the history payload size of '35' bytes and cannot be resized. -> Function can_change_be_added_nts
[INFO] [1663728627.215986803] [minimal_publisher]: send: Hello ROS2 1 
[INFO] [1663728628.207130730] [minimal_publisher]: send: Hello ROS2 2 
[INFO] [1663728629.207069968] [minimal_publisher]: send: Hello ROS2 3 
[INFO] [1663728630.207108352] [minimal_publisher]: send: Hello ROS2 4 

```

# References
- [Unlocking the potential of Fast DDS middleware ](https://docs.ros.org/en/humble/Tutorials/Advanced/FastDDS-Configuration.html#create-the-xml-file-with-the-profile-configuration)
- [Unable to create publishers/subscribers with different profiles #554 ](https://github.com/ros2/rmw_fastrtps/issues/554)

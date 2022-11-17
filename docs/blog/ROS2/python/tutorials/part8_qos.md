---
title: Part8 - QoS
description: ROS2 QoS allows users to fine-tune the transmission behavior according to their own applications
date: "2022-04-29"
banner: ../ros2.png
tags:
    - ros2
    - qos
    - 101
---
Topics in ROS2 have three dimensions:

- **Name**: string
- **Type**: msg type like std_msgs/String
- **QoS**: Define extra promises about the pub/sub behavior

---

## QoS
- **QoS Policy**: QoS "type" or "setting"
- **QoS Profile**: A complete group of all policies
- **QoS Offer**:
- **QoS Request**:
- **Compatibility**: 

```bash title="check topic qos settings"
ros2 topic info --verbose </topic name>
```

---

### Policy

| **Policy**  | **Description**  |
|---|---|
| History  | ==Keep last==: only store up to N samples, configurable via the queue depth option.<br> ==Keep all==: store all samples, subject to the configured resource limits of the underlying middleware.  |
| Depth  | Depth of history queue when specifying **Keep last**  |
| Reliability  | ==Best effort==: attempt to deliver samples, but may lose them if the network is not robust.<br>==Reliable==: guarantee that samples are delivered, may retry multiple times.  |
| Durability  | ==Transient local==: the publisher becomes responsible for persisting samples for “late-joining” subscribers.<br>==Volatile==: no attempt is made to persist samples.  |
| Deadline  | ==Duration==: the expected maximum amount of time between subsequent messages being published to a topic  |
| Lifespan  | how long the sent message can live  |
| Liveliness  | Liveliness sets the Lease Duration, and the publisher is considered offline after a certain time <br>==Automatic==<br>==Manual by topic==  |

---

### Profile
A QoS profile defines a set of policies that are expected to go well together for a particular use case.  
for example:

- service_default
- sensor_data
- parameters
- system_default

[more info check ROS2 document](https://docs.ros.org/en/foxy/Concepts/About-Quality-of-Service-Settings.html)

#### Sensor data
For sensor data, in most cases it’s more important to receive readings in a timely fashion, rather than ensuring that all of them arrive. That is, developers want the latest samples as soon as they are captured, at the expense of maybe losing some. For that reason the sensor data profile uses best effort reliability and a smaller queue size.

profile define `rmw_qos_profile_t`


::: row cards

    ::: col

        ::: card

            ```cpp title="sensor_data"
            static const rmw_qos_profile_t rmw_qos_profile_sensor_data =
            {
            RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            5,
            RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            RMW_QOS_POLICY_DURABILITY_VOLATILE,
            RMW_QOS_DEADLINE_DEFAULT,
            RMW_QOS_LIFESPAN_DEFAULT,
            RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
            RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
            false
            };
            ```

    ::: col

        ::: card
            **fields list**

            - history
            - depth
            - reliability
            - durability
            - deadline
            - lifespan
            - liveliness
            - liveliness_lease_duration
            - avoid_ros_namespace_conventions


            [doc](https://docs.ros2.org/foxy/api/rmw/structrmw__qos__profile__t.html)


### Demos
#### sub code pub cli

- Subscriber Node defined with `sensor data` QoS
- Pub cli to publish message with compatibility QoS


```python title="subscriber"
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

class MinimalSubQoS(Node):
    def __init__(self) -> None:
        super().__init__("minimal_sub_qos")
        
        self.__sub = self.create_subscription(
            String,
            "topic",
            self.__cb,
            qos_profile_sensor_data
        )
        self.__sub

    def __cb(self, msg:String):
        self.get_logger().info(msg.data)


def main(args=None):
    rclpy.init(args=args)
    sub_node = MinimalSubQoS()
    rclpy.spin(sub_node)
    sub_node.destroy_node()
    rclpy.shutdown()
```

```bash title="pub cli"
ros2 topic pub -1 --qos-profile sensor_data /topic std_msgs/msg/String "data: hello"
```

#### pub code sub cli

```python title="publisher"
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

class MinimalPubQos(Node):
    def __init__(self) -> None:
        super().__init__("minimal_pub_qos")
        
        self.__pub = self.create_publisher(
            String,
            "topic",
            qos_profile_sensor_data
        )
        self.__timer = self.create_timer(
            timer_period_sec=0.5,
            callback=self.__timer_cb)
        self.__counter = 0

    def __timer_cb(self):
        msg = String()
        msg.data = "Hello QoS {}".format(self.__counter)
        self.__pub.publish(msg)
        self.__counter += 1

def main(args=None):
    rclpy.init(args=args)
    pub_node = MinimalPubQos()
    rclpy.spin(pub_node)
    pub_node.destroy_node()
    rclpy.shutdown()
```

```bash title="cli echo topic"
# work
ros2 topic echo --qos-profile sensor_data /topic

# work
ros2 topic echo  /topic

# not work
ros2 topic echo --qos-profile services_default /topic

# work
ros2 topic echo --qos-reliability best_effort /topic
```

---

## QoS compatibility
In order for Publisher and Subscriber to establish a connection, the QoS set by the two must be compatible. DDS adopts the Request-Offer model. In short, the communication level provided by Publisher must be greater than or equal to that required by Subscriber. For detailed compatibility table, please refer to [ROS2 official document](https://docs.ros.org/en/foxy/Concepts/About-Quality-of-Service-Settings.html#qos-compatibilities)

---

# Reference
- [Profiling ROS2](https://hackmd.io/@st9540808/r1zrNKBWU/%2F%401IzBzEXXRsmj6-nLXZ9opw%2FBkaxoWRiI)




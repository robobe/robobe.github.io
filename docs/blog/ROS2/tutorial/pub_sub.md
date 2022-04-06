---
title: Part1 - Simple PUB / SUB
description: Write simple ROS2 python package with publisher subscriber and usage ROS2 CLI tools
date: "2022-04-05"
banner: ros2.png
tags:
    - ros2
    - pub
    - sub
    - 101
---
# Publisher
## node source code

<details><summary>Node source code</summary>
    ```python
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    class MinimalPublisher(Node):
        def __init__(self):
            super().__init__("minimal_publisher")
            self.publisher_ = self.create_publisher(String, "minimal", 10)
            timer_period = 1  # seconds
            self.timer = self.create_timer(timer_period, self.timer_callback)
            self.i = 0

        def timer_callback(self):
            msg = String()
            msg.data = f"pub simple: {self.i}"
            self.publisher_.publish(msg)
            self.get_logger().info(f'Publishing: "{msg.data}"')
            self.i += 1


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
</details>


## setup.py
- Add entry point `<node name>.<module name>:<entry func>`

```python
entry_points={
        'console_scripts': [
            "simple_pub = basic.simple_pub:main"
        ],
    }
```

---

## package.xml
- Add this lines before `<export>` tag

```xml
  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
```

!!! Note
    `exec_depend` Declares a rosdep key or ROS package name that this package needs at execution-time

---

## build and run

```bash title="Build"
colcon build --symlink-install --packages-select basic
```

```bash title="Source it"
source install/setup.bash
```

```bash title="Run"
ros2 run basic simple_pub
```

```bash title="output"
[INFO] [1649181441.732282742] [minimal_publisher]: Publishing: "pub simple: 0"
[INFO] [1649181442.713229723] [minimal_publisher]: Publishing: "pub simple: 1"
[INFO] [1649181443.713351778] [minimal_publisher]: Publishing: "pub simple: 2"
```

---
## cli
### ros2 topic
```bash
# list
ros2 topic list
#result
/parameter_events
/rosout
/minimal

# info
ros2 topic info /minimal
# Result
Type: std_msgs/msg/String
Publisher count: 1
Subscription count: 0

# Echo
ros2 topic echo /minimal
# Result
data: 'pub simple: 110'
---
data: 'pub simple: 111'
---

```

#### topic verbose info

```bash title="verbose info" linenums="1" hl_lines="11 12 13"
ros2 topic info -v /minimal
# Result
Type: std_msgs/msg/String

Publisher count: 1

Node name: minimal_publisher
Node namespace: /
Topic type: std_msgs/msg/String
Endpoint type: PUBLISHER
GID: 01.0f.d1.c7.10.ba.23.e1.01.00.00.00.00.00.11.03.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: RMW_QOS_POLICY_RELIABILITY_RELIABLE
  Durability: RMW_QOS_POLICY_DURABILITY_VOLATILE
  Lifespan: 2147483651294967295 nanoseconds
  Deadline: 2147483651294967295 nanoseconds
  Liveliness: RMW_QOS_POLICY_LIVELINESS_AUTOMATIC
  Liveliness lease duration: 2147483651294967295 nanoseconds

Subscription count: 0

```

---

# Subscriber
## node source code
<details><summary>MinimalSubscriber Node source code</summary>
    ```python
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String


    class MinimalSubscriber(Node):
        def __init__(self):
            super().__init__("minimal_subscriber")

            # The node subscribes to messages of type std_msgs/String,
            # over a topic named: /minimal
            # The callback function is called as soon as a message is received.
            # The maximum number of queued messages is 10.
            self.subscription = self.create_subscription(
                String, "minimal", self.__sub_callback, 10
            )

        def __sub_callback(self, msg):
            self.get_logger().info(f"I heard: {msg.data}")


    def main(args=None):
        rclpy.init(args=args)
        minimal_subscriber = MinimalSubscriber()
        rclpy.spin(minimal_subscriber)
        minimal_subscriber.destroy_node()
        rclpy.shutdown()


    if __name__ == "__main__":
        main()
    ```
</details>


## setup.py
- Add entry point

```python
entry_points={
        'console_scripts': [
            "simple_sub = basic.simple_sub:main"
        ],
    }
```

---

## build and run

```bash title="Build"
colcon build --symlink-install --packages-select basic
```

---

## cli
- Run subscriber node



```bash title="Terminal1"
ros2 run basic simple_sub
# Result after pub from terminal 2
[INFO] [1649213924.055190916] [minimal_subscriber]: I heard: hello
[INFO] [1649213925.036938799] [minimal_subscriber]: I heard: hello
```

```bash title="Terminal2"
# pub message
ros2 topic pub /minimal std_msgs/msg/String "{data: 'hello'}"

# pub only one message
ros2 topic pub -1 /minimal std_msgs/msg/String "{data: 'hello'}"
```

---

# References
- [package.xml specification](https://ros.org/reps/rep-0149.html)
- [ros2 topic Command Line Tool – Debug ROS2 Topics From the Terminal](https://roboticsbackend.com/ros2-topic-cmd-line-tool-debug-ros2-topics-from-the-terminal/)
- [Create a Basic Publisher and Subscriber (Python) | ROS2 Foxy](https://automaticaddison.com/create-a-basic-publisher-and-subscriber-python-ros2-foxy/)
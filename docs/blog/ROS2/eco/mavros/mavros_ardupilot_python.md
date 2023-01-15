---
title: Ardupilot MAVROS minimal python node
tags:   
    - ardupilot
    - mavros
---

Implement ROS2 Python node usage `mavros`
That change mode, arm and takeoff using mavros services and topics

## SITL

```bash title="terminal1"
sim_vehicle.py -v ArduCopter --console
```

## mavros
```bash title="terminal2"
ros2 run  mavros mavros_node --ros-args -p fcu_url:=udp://:14550@
```

## node
- set mode to 'GUIDED'
- Arming
- Takeoff


```python
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL
# from mavros_msgs.srv._set_mode import SetMode_Request


TOPIC_TAKEOFF = "/mavros/cmd/takeoff"
TOPIC_ARMING = "/mavros/cmd/arming"
TOPIC_SET_MODE = "/mavros/set_mode"
TOPIC_STATE = "/mavros/state"

class MyNode(Node):
    def __init__(self):
        node_name="minimal"
        super().__init__(node_name)
        self.armed = False
        self.in_air = False
        self.state_sub = self.create_subscription(State, TOPIC_STATE, self.state_cb, qos_profile=qos_profile_system_default)
        self.set_mode_client = self.create_client(SetMode, , qos_profile=qos_profile_system_default)
        self.arming_client = self.create_client(CommandBool, TOPIC_ARMING, qos_profile=qos_profile_system_default)
        self.takeoff_client = self.create_client(CommandTOL, TOPIC_TAKEOFF, qos_profile=qos_profile_system_default)
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('set_mode service not available, waiting again...')
        self.get_logger().info("hello mavros python")

    def mode_changed_handler(self, future):
        response = future.result()
        self.get_logger().info("{}".format(response))

    def arming_handler(self, future):
        response = future.result()
        self.get_logger().info("{}".format(response))
        self.armed = response.success

    def takeoff_handler(self, future):
        response = future.result()
        self.get_logger().info("{}".format(response))
        self.in_air = response.success

    def state_cb(self, msg):
        self.get_logger().info("{}".format(msg), throttle_duration_sec=2)
        if msg.mode == "STABILIZE":
            request = SetMode.Request() # SetMode_Request()
            request.custom_mode = "GUIDED"
            future = self.set_mode_client.call_async(request)
            future.add_done_callback(self.mode_changed_handler)

        if msg.mode == "GUIDED" and not self.armed:
            request = CommandBool.Request()
            request.value = True
            future = self.arming_client.call_async(request)
            future.add_done_callback(self.arming_handler)

        if msg.armed and not self.in_air:
            request = CommandTOL.Request()
            request.altitude = 15.0
            future = self.takeoff_client.call_async(request)
            future.add_done_callback(self.takeoff_handler)


def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
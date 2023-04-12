---
tags:
    - ardupilot
    - rangefinder
    - pymavlink
    - mavros
    - gazebo
    - SITL
    - distance sensor
---
# Add rangefinder using SITL and gazebo 

Distance / Rangefinder has two type of message  

    - Distance sensor #132 (common)
    - Rangefinder #173 (ardupilotmega) (reporting)
  
## Distance sensor #132
Distance sensor mavlink message
- [Distance sensor #132](https://mavlink.io/en/messages/common.html#DISTANCE_SENSOR)

### Demo (using pymavlink)
- Config SITL with RANGE FINDER
- Send mavlink distance using `pymavlink`

```ini  title="params"
RNGFND1_TYPE 10             # mavlink
RNGFND1_ORIENT 25           # down
RNGFND1_MAX_CM 1000         # cm
RNGFND1_MIN_CM 10           # cm
```

```python title="script"
import time

from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega

TO_MS = 1e3
UPDATE_RATE = 0.5
RNGFND_TYPE_MAVLINK = 10
SENSOR_ID = 1
SENSOR_MAX_CM = 1000
SENSOR_MIN_CM = 10
SENSOR_COVARIANCE = 0

SIM_CURRENT_READING_CM = 200

# Create the connection
master = mavutil.mavlink_connection("udp:127.0.0.1:14550")
# Wait a heartbeat before sending commands
master.wait_heartbeat()

t_start = time.time()
while True:
    time.sleep(UPDATE_RATE)
    boot_time = int((time.time() - t_start) * TO_MS)
    master.mav.distance_sensor_send(
        boot_time,
        SENSOR_MIN_CM,
        SENSOR_MAX_CM,
        SIM_CURRENT_READING_CM,
        ardupilotmega.MAV_DISTANCE_SENSOR_UNKNOWN,
        SENSOR_ID,
        ardupilotmega.MAV_SENSOR_ROTATION_PITCH_270,
        SENSOR_COVARIANCE,
    )
```



```bash title="SITL/ sim_vehicle"
./sim_vehicle.py -v ArduCopter \
-f quad -D \
--console \
--add-param-file /home/user/apm_ws/src/apm_bringup/config/range_finder.parm
```

!!! tip "params file"
    Add param file to sitl
    ```
    --add-param-file <file path>
    ```
     
---


## MAVROS
Mavros has two plugins

- distance_sensor (common msg #132)
- rangefinder (ardupilot msg #173)


### Demo
- Send distance using distance_sensor plugin (msg #132)
  - mavros open subscriber for each distance sensor declare as `subscriber: true` see [config](#config-multiple-sensors) file example
- Read data using rangefinder plugin (msg #173)
- Echo distance data from fcu using cli (/mavros/rangfinder_pub)

#### publish distance data
```python title="demo send random distance data"
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range

TOPIC_DISTANCE_READ = "/mavros/rangefinder_pub"
TOPIC_DISTANCE_WRITE = "/mavros/rangefinder_sub"
TOPIC_APM_RANGEFINDER = "/mavros/rangefinder/rangefinder"
DRONE_NO = 1
MIN_RANGE = 0.0
MAX_RANGE = 4.0
RANGE_SENSOR_TYPE = 1
SENSOR_ID = 1
COVARIANCE = 0

PUB_INTERVAL = 1/10

class RangeFinderNode(Node):
    def __init__(self):
        node_name = "range_finder"
        super().__init__(node_name)

        #mavros open subscriber, our node pub to it
        self.__range_pub = self.create_publisher(
            Range, TOPIC_DISTANCE_WRITE, qos_profile=qos_profile_sensor_data
        )
        self.create_subscription(
            Range, TOPIC_APM_RANGEFINDER, self.__apm_rangefinder_message_handler, qos_profile=qos_profile_sensor_data
        )

        self.create_timer(PUB_INTERVAL, self.__send_range_message)

    def __apm_rangefinder_message_handler(self, msg: Range):
        self.get_logger().info(f"apm rangefinder: {msg.range}")

    def __send_range_message(self, distance=2.0):
        sec, nanosec = self.get_clock().now().seconds_nanoseconds()
        range_msg = Range()
        range_msg.header.frame_id = "rangefinder"
        range_msg.header.stamp.sec = sec
        range_msg.header.stamp.nanosec = nanosec
        range_msg.range = float(distance)
        range_msg.radiation_type = RANGE_SENSOR_TYPE
        range_msg.min_range = MIN_RANGE
        range_msg.max_range = MAX_RANGE
        self.__range_pub.publish(range_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RangeFinderNode()
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

#### echo distance data
```bash
ros2 topic echo /mavros/rangefinder_pub
```

!!! tip "config parma"
    `config` param expend settings as YAML
     

#### config multiple sensors


```yaml linenums="1" hl_lines="11 12 15 16"
/mavros/**/distance_sensor:
  ros__parameters:
    config: |
      rangefinder_pub:
        id: 0
        frame_id: "lidar"
        #orientation: PITCH_270 # sended by FCU
        field_of_view: 0.0  # XXX TODO
        send_tf: false
        sensor_position: {x:  0.0, y:  0.0, z:  -0.1}
      rangefinder_sub:
        subscriber: true
        id: 1
        orientation: PITCH_270
      rangefinder_fwd:
        subscriber: true
        id: 2
        orientation: PITCH_180
```

!!! tip "orientation"
     declare at `/mavros/src/lib/enum_sensor_orientation.cpp`
---

# Demo to delete
```bash title="sitl"
sim_vehicle.py -v ArduCopter -f gazebo-iris -A "--defaults /home/user/wasp_ws/src/wasp_bringup/config/copter.parm,/home/user/wasp_ws/src/wasp_bringup/config/gazebo-iris.parm" -I0 -m "--out=127.0.0.1:14552" -m "--load-module graph"
```

![](images/gazebo_sitl_mavproxy_graph_qgc.png)

---
tags:
    - micro ros
    - ros2
    - esp32
---

# Esp32 with micro ROS tutorial

```
ros2 run micro_ros_setup create_firmware_ws.sh freertos esp32
```

```
ros2 run micro_ros_setup configure_firmware.sh int32_publisher --transport serial
```

```
ros2 run micro_ros_setup build_firmware.sh

```

```
ros2 run micro_ros_setup flash_firmware.sh
```

## agent

```
sudo apt install libtinyxml2-dev
```

```
ros2 run micro_ros_setup create_agent_ws.sh
```

```
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash
```

```
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
```

## Usage

Run agent

```
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
```

reboot device or reconnect 

![](images/serial_agent.png)


```bash
ros2 topic list
#
/freertos_int32_publisher
/parameter_events
/rosout

```

```bash
ros2 topic echo /freertos_int32_publisher 
#
data: 0
---
data: 1
---
data: 2
---
data: 3

```
---

## Reference
- [Esp32 with micro ROS tutorial | Int Publisher | error solved | Part 01](https://www.youtube.com/watch?v=fo5I9ZYbG5Q)
- [Connect ESP32 to ROS2 (Foxy +)](https://medium.com/@SameerT009/connect-esp32-to-ros2-foxy-5f06e0cc64df)


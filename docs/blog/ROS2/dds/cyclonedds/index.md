---
tags:
    - ros2
    - cyclonedds
    - dds
    - rmw
---

# Install cyclonedds rmw for ros humble

## install
```
sudo apt install ros-humble-rmw-cyclonedds-cpp
```

## usage

```
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2 run demo_nodes_cpp talker
```

```
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2 run demo_nodes_cpp listener
```

### shm
[Using Shared Memory with ROS 2](https://github.com/ros2/rmw_cyclonedds/blob/humble/shared_memory_support.md)


```
export CYCLONEDDS_URI=file://$PWD/cyclonedds.xml
```
```bash
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2 run demo_nodes_cpp talker
2024-01-16 18:17:20.067 [Warning]: RouDi not found - waiting ...
```

#### introspection
```
apt search ros-humble-iceoryx-introspection
```

---

- [Setting up Node Discovery Across Multiple Systems in ROS2 Infrastructure](https://medium.com/@arshad.mehmood/setting-up-node-discovery-across-multiple-systems-in-ros2-infrastructure-a1a5c25f052f)

---

## Resource to read
- [Configuration File Reference](https://cyclonedds.io/docs/cyclonedds/latest/config/config_file_reference.html#cyclonedds-domain-discovery-peers)
- [cyclonedds github](https://github.com/eclipse-cyclonedds/cyclonedds)
- [2021 Eclipse Cyclone DDS ROS Middleware Evaluation Report with iceoryx and Zenoh](https://osrf.github.io/TSC-RMW-Reports/humble/eclipse-cyclonedds-report.html)
- [ROS 2 Middleware (RMW) Configuration](https://iroboteducation.github.io/create3_docs/setup/xml-config/)
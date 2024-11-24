---
tags:
    - zenoh
    - ros2
    - bridge
    - zenoh-plugin-ros2dds
---


!!! note "zenoh-plugin-dds"
    There is zenoh bridge for DDS, zenoh-plugin-ros2dds bring more advanced integration to ROS2 system



!!! warning "cyclondds"
    The bridge relies on CycloneDDS and has been tested with **RMW_IMPLEMENTATION=rmw_cyclonedds_cpp**. While the DDS implementations are interoperable over UDP multicast and unicast, some specific and non-standard features of other DDS implementations (e.g. shared memory) might cause some issues.

     
## usage
- ubuntu 22.04
- ROS2 humble
- rpi4
- PC
 
 
### install

#### Install on pc and rpi

```bash title="cyclonedds_rmw"
sudo apt install ros-humble-rmw-cyclonedds-cpp
```

```bash title="demo ros package"
sudo apt install ros-humble-demo-nodes-cpp
```

#### Download bridge
[download current release v0.10.1-rc](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/releases/tag/0.10.1-rc)

- [x86_64-unknown-linux-gnu](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/releases/download/0.10.1-rc/zenoh-plugin-ros2dds-0.10.1-rc-x86_64-unknown-linux-gnu.zip)
- [aarch64-unknown-linux-gnu](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/releases/download/0.10.1-rc/zenoh-bridge-ros2dds-0.10.1-rc-aarch64-unknown-linux-gnu.zip)


### Run 
Config system to localhost only [check](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds#usage)

Run terminal (Terminator) and split to 4 pane

#### Robot
```bash title="bridge-robot(rpi4)"
# ssh
./zenoh-bridge-ros2dds -l udp/0.0.0.0:7447 -d 2 --ros-localhost-only
```

```bash title="service-robot(rpi4)"
# ssh
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=2 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2 run demo_nodes_cpp add_two_ints_server
```

#### Home
```bash title="bridge"
./zenoh-bridge-ros2dds -d 1 --rest-http-port 8000  --ros-localhost-only
```

```bash title="service client"
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2 run demo_nodes_cpp add_two_ints_client_async

```


### Admin space
[Check](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds#admin-space)

!!! note "bride id"
    Take hex guid from bridge log
     
---




## Resource
- [zenoh-plugin-ros2dds github](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds)
     
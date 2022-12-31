---
title: ROS2 FastDDS static discovery
tags:
    - dds
    - fast-dds
    - rmw
---

```bash title="terminal1"
source /opt/ros/humble/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE="$(pwd)/STATIC_FASTRTPS_PROFILE_WRITER.xml"
ros2 run demo_nodes_cpp talker
```

```bash title="terminal2"
source /opt/ros/humble/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE="$(pwd)/STATIC_FASTRTPS_PROFILE_READER.xml"
ros2 run demo_nodes_cpp listener
```


---

# Reference
- [Static Discovery Support](https://github.com/ros2/rmw_fastrtps/issues/617)
- [files](examples/fast_dds/static_discovery/)
- [fast-dds discovery](https://fast-dds.docs.eprosima.com/en/latest/fastdds/discovery/discovery.html)
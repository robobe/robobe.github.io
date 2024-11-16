---
tags:
    - ros
    - composition
    - intra-process
    - rclcpp
---

# ROS2 Composition

ROS composition combines multiple nodes (ROS components) into a single process, enhancing performance and efficiency.


```
ros2 pkg create --build-type ament_cmake my_composable_demo --dependencies rclcpp rclcpp_components
```


---

## Reference
- [Composable Nodes in ROS2](https://roscon.ros.org/2019/talks/roscon2019_composablenodes.pdf)
---
title: Minimal ROS2 Node
tags:
    - cpp
    - node
---

## Minimal Node

```cpp title="minimal.cpp"
#include "rclcpp/rclcpp.hpp"

class Minimal : rclcpp::Node
{
public:
  Minimal():Node("Minimal")
  {
    RCLCPP_INFO(this->get_logger(), "hello minimal node");
  }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Minimal>();
    rclcpp::shutdown();
    return 0;
}
```

```c title="CMakeLists.txt"
cmake_minimum_required(VERSION 3.8)
project(cpp_tutorial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(fmt REQUIRED)

add_executable(minimal src/minimal.cpp)
ament_target_dependencies(minimal rclcpp fmt:fmt)

install(TARGETS
  minimal
  DESTINATION lib/${PROJECT_NAME})

ament_package()
```

---
### usage

```bash
source install/setup.bash
ros2 run cpp_tutorial minimal
```

### logging

!!! tip "Write log to file"
    Write log file to file [ROS2 Demo](https://docs.ros.org/en/humble/Tutorials/Demos/Logging-and-logger-configuration.html#logging-directory-configuration)

    ```bash
    mkdir /tmp/ros_log
    export ROS_LOG_DIR=/tmp/ros_log
    ros2 run cpp_tutorial minimal
    ```
     
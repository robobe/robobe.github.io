---
title: Minimal ROS2 Node
tags:
    - cpp
    - node
    - logging
---

Create minimal node with logging

## Create package
```bash
ros2 pkg create <package_name> --build-type ament_cmake --dependencies <package_dependencies>

```

## Minimal Node

!!! note "logger"
  - node level log: get the node name as logger source
  - rclcpp level log: user set the logger name
     
```cpp title="minimal.cpp"
#include "rclcpp/rclcpp.hpp"

rclcpp::Logger mlog = rclcpp::get_logger("simple_pub");

class Minimal : rclcpp::Node
{
public:
  Minimal() : public Node("Minimal")
  {
    RCLCPP_INFO(this->get_logger(), "hello ros2 cpp node");
    RCLCPP_WARN(rclcpp::get_logger("my_logger"), "logging from rclcpp");
    RCLCPP_ERROR(mlog, "error logging");
    RCLCPP_INFO_STREAM(mlog, "a"
                                 << "b"
                                 << "c");
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

!!! tip "copy/install binaries"
     ```
    install(TARGETS
      <binary name/target>
    DESTINATION lib/${PROJECT_NAME}
    )
     ```


!!! tip "copy/install folders"
     ```
    install(DIRECTORY
      launch
    DESTINATION share/${PROJECT_NAME}
    )
     ```
---

### build

```bash
# from WS root folder
colcon build --package-select <package name>
```
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
     
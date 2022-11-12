---
title: ROS2 pkg with python and cpp
tags:
    - ros2
---

# Objective
- Create pkg with python and cpp code


# Project
- Create new pkg `cmake_ament`
- Add python `src` folder name like the package name
- Add `__init__.py` file to mark this folder as a python package


```
mix_pkg
│
├── CMakeLists.txt
├── include
│   └── mix_pkg
├── mix_pkg
│   ├── __init__.py
│   └── simple_sub.py
├── package.xml
└── src
    └── simple_pub.cpp
```

## CMake
The pkg is a standard `cmake` pkg with python support

- Add `ament_cmake_python` package to use
- Use `ament_python_install_package` macro from this pkg to install the python files
- Copy the python scripts to `pkg` `install\<project_name>\lib folder



```c
cmake_minimum_required(VERSION 3.8)
project(mix_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(simple_pub src/simple_pub.cpp)
ament_target_dependencies(simple_pub rclcpp std_msgs)

# Install Cpp executables
install(TARGETS
  simple_pub
	DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
	mix_pkg/simple_sub.py
	DESTINATION lib/${PROJECT_NAME}
	)

ament_package()

```

## package.xml
- Add build and other dependencies


```xml
<buildtool_depend>ament_cmake</buildtool_depend>
<buildtool_depend>ament_cmake_python</buildtool_depend>

<depend>rclcpp</depend>
<depend>rclpy</depend>
```

## code
```python title="mix_pkg/simple_sub.py"
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimpleSub(Node):
    def __init__(self):
        super().__init__("simple_sub_py")
        self.subscription = self.create_subscription(
            String, "topic", self.listener_callback, 10
        )
        self.subscription

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleSub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

```

```cpp title="src/simple_pub.cpp"
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class SimplePublisher : public rclcpp::Node
{
public:
    SimplePublisher() : Node("simple_cpp_pub"), count_(0)
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        timer_ = this->create_wall_timer(
            500ms, std::bind(&SimplePublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimplePublisher>());
    rclcpp::shutdown();
    return 0;
}
```

---

# Build and usage

!!! warning 
    cmake_python_ament set the Python permission script under install/lib/<pkg_name>
    If we build the package with   `--symlink-install` no permission has set


## usage

```bash title="terminal 1"
ros2 run mix_pkg simple_pub
```

```bash title="terminal 2"
ros2 run mix_pkg simple_sub.py
```

!!! note
    Run the python node with the `py` extension

---

# Reference
- [ament_cmake_python](https://docs.ros.org/en/humble/How-To-Guides/Ament-CMake-Python-Documentation.html)
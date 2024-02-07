---
tags:
    - ros
    - dds
    - cyclonedds
    - zero-copy
    - shm
    - loan message
---

# Using cyclonedds with SHM to implement zero copy
[must read before continue](https://github.com/ros2/rmw_cyclonedds/blob/master/shared_memory_support.md)


## Simple demo
- ubuntu 22.04
- humble
- cyclonedds

### pub
- using loan message
  
```cpp linenums="1" hl_lines="26 27"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <rclcpp/qos.hpp>
#include <chrono>
#include <string>

using namespace std::chrono_literals;

const std::string TOPIC = "/topic";

class Minimal : public rclcpp::Node
{
public:
  Minimal() : Node("Minimal")
  {
    auto qos = rclcpp::SensorDataQoS();
    timer_ = this->create_wall_timer(1s, std::bind(&Minimal::timer_handler, this));
    pub_ = this->create_publisher<std_msgs::msg::UInt32>(TOPIC, qos);
    RCLCPP_INFO(this->get_logger(), "minimal pub with loan message");
  };

private:
  void timer_handler()
  {
    // auto msg = std::make_unique<std_msgs::msg::UInt32>();
    auto msg = this->pub_->borrow_loaned_message();
    msg.get().data = counter_++;
    pub_->publish(std::move(msg));
  }

  int counter_ = 0;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Minimal>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

### sub

```cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <rclcpp/qos.hpp>
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

const std::string TOPIC = "/topic";

class Minimal : public rclcpp::Node
{
public:
    Minimal() : Node("MinimalSub")
    {
        auto qos = rclcpp::SensorDataQoS();
        sub = this->create_subscription<std_msgs::msg::UInt32>(
            TOPIC,
            qos,
            std::bind(&Minimal::topic_callback, this, _1));
    }

private:
    void topic_callback(const std_msgs::msg::UInt32::SharedPtr msg)
    {
        RCLCPP_INFO_STREAM(this->get_logger(), msg->data);
    }
    rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr sub;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Minimal>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

### CMakeLists.txt

```bash
cmake_minimum_required(VERSION 3.8)
project(cpp_demos)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)


add_executable(simple_sub src/simple_sub.cpp)
ament_target_dependencies(simple_sub rclcpp std_msgs)

add_executable(simple_pub src/simple_pub.cpp)
ament_target_dependencies(simple_pub rclcpp std_msgs)

install(TARGETS
  simple_sub
  simple_pub
  DESTINATION lib/${PROJECT_NAME})

ament_package()
```

## usage

```xml title="cyclonedds.xml"
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/iceoryx/etc/cyclonedds.xsd">
    <Domain id="any">
        <SharedMemory>
            <Enable>true</Enable>
            <LogLevel>info</LogLevel>
        </SharedMemory>
    </Domain>
</CycloneDDS>
```

```bash title="run roudi"
iox-roudi
```

```bash
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2 run cpp_demos simple_pub
```

```bash
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2 run cpp_demos simple_sub
```
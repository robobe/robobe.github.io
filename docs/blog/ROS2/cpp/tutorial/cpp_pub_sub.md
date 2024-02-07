---
tags:
    - ros
    - cpp
    - pub
    - sub
---
# Minimal ROS2 pub sub node

```cpp
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"

#include <rmw/qos_profiles.h>
#include <rclcpp/qos.hpp>
#include <rclcpp/time.hpp>
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/image.hpp>

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
      : Node("minimal_publisher")
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
    // auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("topic", qos);
    timer_ = this->create_wall_timer(
        1000ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    rclcpp::Time now = this->get_clock()->now();
    auto message = sensor_msgs::msg::Image();
    message.header.stamp = now;
    std::this_thread::sleep_for(1s);
    this->publisher_->publish(message);
    // RCLCPP_INFO_STREAM(this->get_logger(), ""<<(c-message.header.stamp).seconds());
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MinimalPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

---

## Sub

```cpp
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/time.hpp"
#include <rmw/qos_profiles.h>

class MinimalSub : public rclcpp::Node
{
public:
    MinimalSub() : Node("minimal_img_sub")
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "hello sub");
        // auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
        sub_ = this->create_subscription<sensor_msgs::msg::Image>("topic", qos, std::bind(&MinimalSub::img_handler, this, std::placeholders::_1));
    }

    

private:
    void img_handler(sensor_msgs::msg::Image::SharedPtr msg)
    {
        auto current = this->get_clock()->now();
        RCLCPP_INFO_STREAM(this->get_logger(), "" << (current - msg->header.stamp).seconds());
    }
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MinimalSub>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```


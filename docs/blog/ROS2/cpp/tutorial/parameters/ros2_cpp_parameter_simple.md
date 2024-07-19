---
tags:
    - rclcpp
    - ros2
    - cpp
    - 
---

# Simple node with parameter

```cpp title="simple_param"
#include "rclcpp/rclcpp.hpp"

class SimpleParam : public rclcpp::Node
{
public:
  SimpleParam() : Node("simple_param")
  {
    this->declare_parameter<int>("my_int", 10);

    int my_param = this->get_parameter("my_int").as_int();

    RCLCPP_INFO_STREAM(this->get_logger(), "param value " << my_param);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimpleParam>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

## usage

```
ros2 run cpp_demos simple_param --ros-args -p my_int:=20
```
---
title: launch with node arguments
tags:
    - ros2
    - launch
---

```cpp title="param_hello.hpp"
#ifndef PARAM_HELLO_HPP
#define PARAM_HELLO_HPP
#include "rclcpp/rclcpp.hpp"

class ParamHelloNode final : public rclcpp::Node
{
public:
  ParamHelloNode();

private:
    std::string param_str_;
};

#endif  // PARAM_HELLO_HPP
```

```cpp title="param_hello.cpp"
#include "param_hello.hpp"
#include "rclcpp/rclcpp.hpp"

ParamHelloNode::ParamHelloNode() : Node("hello_param")
{
    RCLCPP_INFO(this->get_logger(), "hello param");
    this->declare_parameter("param_name", "hello_default");
    this->param_str_ = this->get_parameter("param_name").as_string();
    RCLCPP_INFO(this->get_logger(), "%s", this->param_str_.c_str());
};
```

```python title="launch node with arguments"
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    node=Node(
        name="simple_param",
        package = 'cpp_tutrial_pkg',
        executable = 'param_hello',
        parameters = [
            {"param_name": "param value from launch"}
        ]
    )
    ld.add_action(node)
    return ld
```

```python title="launch with arguments from cli"
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ld = LaunchDescription()
    param_name = LaunchConfiguration('param_name')

    arg_cmd = DeclareLaunchArgument("param_name", default_value="hello world", description="msg simple description")

    node=Node(
        name="simple_param",
        package = 'cpp_tutrial_pkg',
        executable = 'param_hello',
        parameters = [
            {"param_name": param_name}
        ]
    )

    ld.add_action(arg_cmd)
    ld.add_action(node)
    return ld

```
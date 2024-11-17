---
tags:
    - ros
    - composition
    - intra-process
    - rclcpp
---

# ROS2 Composition

ROS composition combines multiple nodes (ROS components) into a single process, enhancing performance and efficiency.

A composable node is any ROS 2 node that supports composition


## Demo: Talker / Listener

```bash title="create a package"
ros2 pkg create --build-type ament_cmake \
my_composable_demo \
--dependencies rclcpp rclcpp_components
```

```bash
my_composable_demo/
├── CMakeLists.txt
├── include
│   └── my_composable_demo
│       ├── listener_component.hpp
│       └── talker_component.hpp
├── launch
│   └── talker_listener.launch.py
├── package.xml
└── src
    ├── listener_component.cpp
    └── talker_component.cpp
```

```cpp title="talker_component.hpp"
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace composition
{

    class Talker : public rclcpp::Node
    {
    public:
        explicit Talker(const rclcpp::NodeOptions &options);

    protected:
        void on_timer();

    private:
        size_t count_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
        rclcpp::TimerBase::SharedPtr timer_;
    };

} // namespace composition
```

```cpp title="listener_component.hpp"
#pragma once
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace composition
{
  class Listener : public rclcpp::Node
  {
  public:
    explicit Listener(const rclcpp::NodeOptions &options);

  private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  };

} // namespace composition
```

```cpp title="talker_component.cpp"
#include "my_composable_demo/talker_component.hpp"
#include <chrono>
#include <iostream>
#include <memory>
#include <utility>

using namespace std::chrono_literals;

namespace composition
{
  Talker::Talker(const rclcpp::NodeOptions &options)
      : Node("talker", options), count_(0)
  {
    pub_ = create_publisher<std_msgs::msg::String>("chatter", 10);
    timer_ = create_wall_timer(1s, std::bind(&Talker::on_timer, this));
  }

  void Talker::on_timer()
  {
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = "Hello World: " + std::to_string(++count_);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg->data.c_str());
    std::flush(std::cout);

    pub_->publish(std::move(msg));
  }

} // namespace composition

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(composition::Talker)
```

```cpp title="listener_component.cpp"
#include "my_composable_demo/listener_component.hpp"

#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace composition
{
Listener::Listener(const rclcpp::NodeOptions & options)
: Node("listener", options)
{
  auto callback =
    [this](std_msgs::msg::String::ConstSharedPtr msg) -> void
    {
      RCLCPP_INFO(this->get_logger(), "I heard: [%s]", msg->data.c_str());
    };

  sub_ = create_subscription<std_msgs::msg::String>("chatter", 10, callback);
}

}  // namespace composition

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(composition::Listener)
```

```bash title="CMakeLists.txt"
cmake_minimum_required(VERSION 3.8)
project(my_composable_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

# talker
set(packages
  "rclcpp"
  "rclcpp_components"
  "std_msgs"
)
add_library(talker_component SHARED
  src/talker_component.cpp)

  ament_target_dependencies(talker_component
    ${packages}
  )
rclcpp_components_register_nodes(talker_component "composition::Talker")

# listener
add_library(listener_component SHARED
  src/listener_component.cpp)

ament_target_dependencies(listener_component
  ${packages}
)
rclcpp_components_register_nodes(listener_component "composition::Listener")


install(TARGETS
  talker_component
  listener_component

  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY
launch
DESTINATION share/${PROJECT_NAME}
)

ament_package()

```

```python title="talker_listener.launch.py"
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    container = ComposableNodeContainer(
            name='my_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='my_composable_demo',
                    plugin='composition::Talker',
                    name='talker'),
                ComposableNode(
                    package='my_composable_demo',
                    plugin='composition::Listener',
                    name='listener')
            ],
            output='screen',
    )

    return launch.LaunchDescription([container])
```

---

### cli

```bash
ros2 run rclcpp_components component_container
```

```bash
# Talker
ros2 component load /ComponentManager my_composable_demo composition::Talker
#
Loaded component 1 into '/ComponentManager' container node as '/talker'

ros2 component load /ComponentManager my_composable_demo composition::Listener
#
Loaded component 2 into '/ComponentManager' container node as '/listener'
```

```bash
ros2 component list
#
/ComponentManager
  1  /talker
  2  /listener
```

```bash title="unload"
ros2 component unload /ComponentManager 2
#
Unloaded component 2 from '/ComponentManager' container node
```

---


### Stand alone
Load composable node from executable

```cpp
#include <rclcpp/rclcpp.hpp>
#include "my_composable_demo/talker_component.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<composition::Talker>(options);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();

  return 0;
}
```

Add executable section to cmake list

```bash
# standalone
add_executable(standalone src/talker_node.cpp)
target_link_libraries(standalone talker_component)
ament_target_dependencies(standalone rclcpp)

# Install the executable
install(TARGETS standalone
  DESTINATION lib/${PROJECT_NAME}
)
```


---

## Reference
- [Composable Nodes in ROS2](https://roscon.ros.org/2019/talks/roscon2019_composablenodes.pdf)
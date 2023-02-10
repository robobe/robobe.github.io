---
title: ROS2 module plugin template
tags:
    - ros2
    - gazebo
    - plugin
---

Module plugin minimal template show how to
- subscribe message from ROS
- publish message to ROS
- logging

The minimal project include two projects
- Plugin (demo_gazebo_plugin)
- Tester (demo_gazebo_plugin_sim)

## Projects

```bash
# Plugin
demo_gazebo_plugin
├── CMakeLists.txt
├── include
│   └── demo_gazebo_plugin
│       └── demo_gazebo_plugin.hpp
├── package.xml
└── src
    └── demo_gazebo_plugin.cpp

# Tester
demo_gazebo_plugin_sim/
├── CMakeLists.txt
├── include
│   └── demo_gazebo_plugin_sim
├── launch
│   └── sim_bringup.launch.py
├── models
│   └── demo_model
│       ├── model.config
│       └── model.sdf
├── package.xml
└── src

```

---

## Plugin

```
ros2 pkg create demo_gazebo_plugin --build-type ament_cmake --dependencies rclcpp std_msgs gazebo_ros
```

```cpp title="demo_gazebo_plugin.hpp"
#ifndef DEMO_GAZEBO_PLUGIN_HPP
#define DEMO_GAZEBO_PLUGIN_HPP

#include <gazebo/common/PID.hh>
#include <gazebo/common/Plugin.hh>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"

namespace demo_gazebo_plugin
{
    class DemoGazeboPlugin : public gazebo::ModelPlugin
    {
    public:
        DemoGazeboPlugin();
        void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf);

    private:
        rclcpp::Node::SharedPtr ros_node_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_pub_;
        void sub_handler(const std_msgs::msg::String::SharedPtr msg);
    };
}

#endif
```

```cpp title="demo_gazebo_plugin.cpp"
#include "demo_gazebo_plugin/demo_gazebo_plugin.hpp"
#include <gazebo_ros/node.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::placeholders;

namespace demo_gazebo_plugin
{   
    const std::string SUB_TOPIC = "/demo_gazebo_cmd";
    const std::string PUB_TOPIC = "/demo_gazebo_echo";
    
    DemoGazeboPlugin::DemoGazeboPlugin()
    {
        
    }

    void DemoGazeboPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf){
        ros_node_ = gazebo_ros::Node::Get(sdf);
        RCLCPP_INFO(ros_node_->get_logger(), "info Demo Gazebo Plugin");
        RCLCPP_WARN(ros_node_->get_logger(), "warning Demo Gazebo Plugin");
        RCLCPP_ERROR(ros_node_->get_logger(), "error Demo Gazebo Plugin");
        // gzmsg << "gz message\n";
        // gzerr << "gz error message\n";

        command_pub_ = ros_node_->create_publisher<std_msgs::msg::String>(PUB_TOPIC, 10);
        command_sub_ = ros_node_->create_subscription<std_msgs::msg::String>(
                SUB_TOPIC,
                10,
                std::bind(&DemoGazeboPlugin::sub_handler, this, _1)
            );
    }

    void DemoGazeboPlugin::sub_handler(const std_msgs::msg::String::SharedPtr msg){
        RCLCPP_WARN(ros_node_->get_logger(), "------ %s ------", msg->data.c_str());
        auto echo_msg = std_msgs::msg::String();
        echo_msg.data = msg->data + "_echo";
        command_pub_->publish(echo_msg);
    }

GZ_REGISTER_MODEL_PLUGIN(DemoGazeboPlugin)
}
```

```c title="CMakeLists.txt"
cmake_minimum_required(VERSION 3.8)
project(demo_gazebo_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_library(demo_gazebo_plugin SHARED src/demo_gazebo_plugin.cpp)

target_include_directories(demo_gazebo_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(demo_gazebo_plugin
  "gazebo_ros"
  "rclcpp"
  "std_msgs")

install(TARGETS
  demo_gazebo_plugin
  DESTINATION share/${PROJECT_NAME})

ament_package()

```

---

## Test Project
```
ros2 pkg create demo_gazebo_plugin --build-type ament_cmake --dependencies demo_gazebo_plugin
```

- Add plugin to model
```xml
<plugin name="demo_gazebo_plugin" filename="libdemo_gazebo_plugin.so"/>
```

### launch
- Set plugin path
- Launch gazebo
- Spawn model

```python title="sim_bringup.launch.py"
from launch import LaunchDescription
import os
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import (
    AppendEnvironmentVariable, 
    DeclareLaunchArgument)
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro


PACKAGE = "demo_gazebo_plugin_sim"
WORLD = "empty.world"
MODEL = "demo_model"

def generate_launch_description():
    ld = LaunchDescription()

    pkg = get_package_share_directory(PACKAGE)
    pkg_plugin = get_package_share_directory("demo_gazebo_plugin")
    gazebo_pkg = get_package_share_directory('gazebo_ros')

    verbose = LaunchConfiguration("verbose")
    arg_gazebo_verbose = DeclareLaunchArgument("verbose", default_value="true")
    world = LaunchConfiguration("world")
    arg_gazebo_world = DeclareLaunchArgument("world", default_value=WORLD)
    sim_time = LaunchConfiguration("sim_time")
    arg_sim_time = DeclareLaunchArgument("sim_time", default_value="true")

    resources = [os.path.join(pkg, "worlds")]

    resource_env = AppendEnvironmentVariable(
        name="GAZEBO_RESOURCE_PATH", value=":".join(resources)
    )

    plugins = [pkg_plugin]

    plugin_env = AppendEnvironmentVariable(
        name="GAZEBO_PLUGIN_PATH", value=":".join(plugins)
    )

    robot_description_path = os.path.join(pkg, "models", MODEL, "model.sdf")
    doc = xacro.parse(open(robot_description_path))
    xacro.process_doc(doc)
    robot_description = doc.toxml()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"use_sim_time": sim_time, "robot_description": robot_description}],
    )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    gazebo_pkg, 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'verbose': verbose, "world": world}.items()
             )
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "demo", "-topic", "robot_description", "-z", "0.0"],
        output="screen",
    )

    ld.add_action(resource_env)
    ld.add_action(plugin_env)
    ld.add_action(arg_gazebo_verbose)
    ld.add_action(arg_gazebo_world)
    ld.add_action(arg_sim_time)
    ld.add_action(robot_state_publisher)
    ld.add_action(gazebo)
    ld.add_action(spawn_entity)
    
    
    return ld
```

```c title="CMakeLists.txt"
cmake_minimum_required(VERSION 3.8)
project(demo_gazebo_plugin_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(demo_gazebo_plugin REQUIRED)


install(DIRECTORY 
  launch
  models
  DESTINATION share/${PROJECT_NAME})


ament_package()

```

---

## Run / Usage

```bash title="terminal1"
# Run gazebo
ros2 launch demo_gazebo_plugin_sim sim_bringup.launch.py
```

```bash title="terminal2"
# pub message to plugin
ros2 topic pub --once /demo_gazebo_cmd std_msgs/String "data: hello"
publisher: beginning loop
publishing #1: std_msgs.msg.String(data='hello')
```

```bash title="terminal2"
# sub message from gazebo
ros2 topic echo /demo_gazebo_echo 
data: hello_echo
---

```
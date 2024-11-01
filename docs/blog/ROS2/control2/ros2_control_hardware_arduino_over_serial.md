---
tags:
    - ros2
    - ros2_control
    - arduino
    - serial
    - hardware
---

# ROS2 Control Arduino Hardware Interface

Implement ros2_control hardware interface that connect arduino board over serial


## project demo

```bash title="project"
├── CMakeLists.txt
├── heavy_cart_hardware.xml
├── include
│   └── heavy_cart_hardware
│       └── heavy_cart_hardware.hpp
├── package.xml
└── src
    └── heavy_cart_hardware.cpp
```

#### interface header and implementation
```cpp title="heavy_cart_hardware.hpp"
#ifndef __HEAVY_CART_HARDWARE__
#define __HEAVY_CART_HARDWARE__

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"

namespace heavy_cart_arduino
{
    class HeavyCartArduinoHardware : public hardware_interface::SystemInterface
    {
        // on_configure;
        // on_cleanup;
        // on_activate;
        // on_deactivate;
        // on_shutdown;
        // on_error;
        public:
        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareInfo &info) override;

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        hardware_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn on_cleanup(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::return_type read(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

        hardware_interface::return_type write(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;
    };
}
#endif
```


```cpp title="heavy_cart_hardware.cpp"
#include <rclcpp/rclcpp.hpp>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "heavy_cart_hardware/heavy_cart_hardware.hpp"
#include <vector>

namespace heavy_cart_arduino
{
    hardware_interface::CallbackReturn HeavyCartArduinoHardware::on_init(
        const hardware_interface::HardwareInfo &info)
    {

        // init serial and other inits

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn HeavyCartArduinoHardware::on_cleanup(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("HeavyCartArduinoHardware"), "Cleaning up ...please wait...");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn HeavyCartArduinoHardware::on_configure(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("HeavyCartArduinoHardware"), "Configuring ...please wait...");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn HeavyCartArduinoHardware::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("HeavyCartArduinoHardware"), "Activating ...please wait...");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn HeavyCartArduinoHardware::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("HeavyCartArduinoHardware"), "Deactivating ...please wait...");
        RCLCPP_INFO(rclcpp::get_logger("HeavyCartArduinoHardware"), "Successfully deactivated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> HeavyCartArduinoHardware::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> HeavyCartArduinoHardware::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        return command_interfaces;
    }

    hardware_interface::return_type HeavyCartArduinoHardware::read(
        const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
    {
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type HeavyCartArduinoHardware::write(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        return hardware_interface::return_type::OK;
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  heavy_cart_arduino::HeavyCartArduinoHardware, hardware_interface::SystemInterface)
```

!!! tip "don't forget: plugin export"
    ```cpp
     #include "pluginlib/class_list_macros.hpp"
        PLUGINLIB_EXPORT_CLASS(
        heavy_cart_arduino::HeavyCartArduinoHardware, hardware_interface::SystemInterface)
     ```

#### Plugins xml

```xml title="heavy_cart_arduino.xml"
<library path="heavy_cart_arduino">
    <class name="heavy_cart_arduino"
           type="heavy_cart_arduino::HeavyCartArduinoHardware"
           base_class_type="hardware_interface::SystemInterface">
      <description>
        The ROS2 Control minimal robot protocol
      </description>
    </class>
</library>
```

### CMakeLists

```bash title="CMakeLists"
cmake_minimum_required(VERSION 3.8)
project(heavy_cart_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

add_library(
  heavy_cart_arduino
  SHARED
  src/heavy_cart_hardware.cpp
)
target_compile_features(heavy_cart_arduino PUBLIC cxx_std_17)
target_include_directories(heavy_cart_arduino PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/heavy_cart_arduino>
)
ament_target_dependencies(
  heavy_cart_arduino PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

install(TARGETS heavy_cart_arduino
  EXPORT export_heavy_cart_arduino
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

pluginlib_export_plugin_description_file(hardware_interface heavy_cart_arduino.xml)
ament_export_targets(export_heavy_cart_arduino HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()

```

!!! tip vscode cpp settings
    Add section to `c_cpp_properties.json`

    ```json title=".vscode/c_cpp_properties.json"
    {
        "configurations": [
            {
                "name": "heavy_cart",
                "includePath": [
                    "/opt/ros/humble/include/**",
                    "/workspaces/heavy_cart_ws/src/heavy_cart_hardware/include/**"
                ],
                "cppStandard": "c++17"
            },
        ]
    }
    ```

---

### URDF

```xml
<?xml version="1.0"?>
<robot>
    <ros2_control name="RealRobot" type="system">
        <hardware>
            <plugin>heavy_cart_arduino</plugin>
        </hardware>
        <joint name="wheel_left_joint">
            <command_interface name="position">
                <param name="min">-10</param>
                <param name="max">10</param>
            </command_interface>
            <state_interface name="position" />
        </joint>
        <joint name="wheel_right_joint">
            <command_interface name="position">
                <param name="min">-10</param>
                <param name="max">10</param>
            </command_interface>
            <state_interface name="position" />
        </joint>
    </ros2_control>


</robot>
```

### Launch

- Run node `ros2_control_node` with robot description and robot_control yaml

!!! note "urdf"
    `ros2_control_node` use urdf to load resource_manager
     
```python
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import xacro
from pathlib import Path


URDF = "turtlebot3_burger.xacro"
PKG_DESCRIPTION = "heavy_cart_description"
PKG_BRINGUP = "heavy_cart_bringup"

def generate_launch_description():
    ld = LaunchDescription()

    pkg_description = get_package_share_directory(PKG_DESCRIPTION)
    xacro_file = Path(pkg_description).joinpath("urdf", URDF).as_posix()
    urdf = xacro.process_file(xacro_file).toxml()

    robot_description = {"robot_description": urdf}
    
    pkg_bringup = get_package_share_directory(PKG_BRINGUP)
    robot_controllers = Path(pkg_bringup).joinpath("config", "heavy.yaml").as_posix()

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )

    ld.add_action(control_node)

    return ld
```
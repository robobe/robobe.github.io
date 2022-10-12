---
title: Part7 - Custom msgs and srvs
description: ROS2 custom messages and service msg
date: "2022-04-08"
banner: ../ros2.png
tags:
    - ros2
    - custom
    - msgs
    - 101
---

!!! Note
    End/Suffix package name with `_interfaces` or `_msgs`

## Create pkg
- Create cpp package
  
  ```bash
  # Default build type ament cmake
  ros pkg create rosmav_msgs
  ```

## package.xml
- Add lines

```xml
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```


## CMakeLists.txt
```cmake title="CMakeLists.txt" linenums="1" hl_lines="14 18"
cmake_minimum_required(VERSION 3.5)
project(my_robot_interfaces)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "your custom interfaces will be here"
  msgs/Attitude.msg
 )

ament_package()
```

## VSCode tips
- Add `msgs` path to `python.analysis.extraPaths`

```json title="pylance"
"python.analysis.extraPaths": [
        "/home/user/dev_ws/install/rosmav_msgs/lib/python3.8/site-packages"
    ]
```

## interface cli
Show information about ROS interfaces

- list: List all interface types available
- package: Output a list of available interface types within one package
- show: Output the interface definition

```bash
ros2 interface  package rosmav_msgs 
# Result
rosmav_msgs/msg/Attitude

# Show 
# --- separate between request response
ros2 interface show std_srvs/srv/Trigger 
---
bool success   # indicate successful run of triggered service
string message # informational, e.g. for error messages

```
---

# References
- [ROS2 Create Custom Message (Msg/Srv)](https://roboticsbackend.com/ros2-create-custom-message/)
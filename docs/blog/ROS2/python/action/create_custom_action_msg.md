---
title: Create custom action msg
tags:
    - ros2
    - python
    - action
---

Actions are defined in `.action` locate in `action` sub folder

Action files defined in this structure

```
# Request
---
# Result
---
# Feedback
```

## Demo
### Action Definition
```title="action/MyAction.action"
int32 count
---
int32 total
---
int32 current
```

### CMakeLists

```c title="CMakeList.txt" linenums="1" hl_lines="11 13"
cmake_minimum_required(VERSION 3.8)
project(action_tutorial_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MyAction.action"
)

ament_package()
```

!!! tip "CMakeLists"
     Keep each action in new line


### Package.xml
```xml title="Add to package.xml"
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<depend>action_msgs</depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

### build and test

```bash
colcon build --packages-select action_tutorial_interfaces 
```

```bash title="check"
ros2 interface show action_tutorial_interfaces/action/MyAction 

#

int32 count
---
int32 total
---
int32 current

```
---

# Resources
- [Creating an action](https://docs.ros.org/en/humble/Tutorials/Intermediate/Creating-an-Action.html)
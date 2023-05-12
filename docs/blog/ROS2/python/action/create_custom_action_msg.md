---
title: Create custom action msg
tags:
    - ros2
    - python
    - action
    - custom messages
---

Actions are defined as `.action` file locate in `action` sub folder
The best practice is to create a package dedicated to ROS2 custom messages.
The package type is `cmake_ament` and 
naming the package with `_interface` suffix so it' clear that the package is an interface package.

```bash
ros2 pkg create custom_interfaces
rm -rf include
rm -rf src
mkdir msg srv action
```

- Remove from the package the `src` and `include` folders
- Create folders for `msg`, `srv`, `action` sub folders/
- Add `action file` for example name it `Counter.action`
- Action files defined in this structure

```
# Request
---
# Result
---
# Feedback
```

### Action Definition
```title="action/Counter.action"
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
ament_export_dependencies(rosidl_default_runtime)

ament_package()
```

!!! tip "CMakeLists"
     Keep each action,srv or msg in new line


### Package.xml
```xml title="Add to package.xml"
  <buildtool_depend>rosidl_default_generators</buildtool_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
```

### build and test
#### build
```bash
colcon build --packages-select action_tutorial_interfaces 
```

#### test
```bash
ros2 interface list | grep Counter
```

```bash title="check"
ros2 interface show custom_interfaces/action/Counter

#

int32 count
---
int32 total
---
int32 current

```

---

## Resources
- [Creating an action](https://docs.ros.org/en/humble/Tutorials/Intermediate/Creating-an-Action.html)
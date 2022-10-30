---
title: ROS2 custom message
tags:
    - msg
---

# LAB
- Create and use custom msg

---

## Custom msg package

```bash title="create pkg"
ros2 pkg create custom_interfaces --build-type ament_cmake \
--dependencies rclcpp
```

```bash title="msg folder"
# create msg folder
mkdir msg 
```

```bash title="msg file"
# create txt file name Age.msg
# more types 
float32 age
```

### cmake
- Add `rosidl_default_generators` package
- add `rosidl_generate_interfaces` macro

```c
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Age.msg"
)

```

### package.xml

- Add the following lines to the package.xml file.

```xml
<build_depend>rosidl_default_generators</build_depend>

<exec_depend>rosidl_default_runtime</exec_depend>

<member_of_group>rosidl_interface_packages</member_of_group>

```

---

## usage

- Create publisher to pub the new msg

### cmake

```c
find_package(custom_interfaces REQUIRED) # This is the package that contains the custom interface

add_executable(age_publisher_node src/publish_age.cpp)
ament_target_dependencies(age_publisher_node rclcpp custom_interfaces) 

install(TARGETS
   age_publisher_node
   DESTINATION lib/${PROJECT_NAME}
 )

```

### package.xml
- Add depend for our new package

```xml
<depend>custom_interfaces</depend>
```

---

# Reference
- [About ROS 2 interfaces](https://docs.ros.org/en/humble/Concepts/About-ROS-Interfaces.html)
---
tags:
    - ros2
    - cpp
    - python
    - mix
    - multi language
---

# Create a ROS2 package for Both Python and Cpp Nodes
Add ROS Python node to ament_cmake package without custom dependencies like other modules

- Add scripts folder for python scripts
- Modify CMakeLists.txt
- Modify package.xml
- Build and test

## Simple Demo
- Ubuntu 22.04 ROS humble

```bash title="create ament_cmake pkg"
ros2 pkg create my_cpp_py_pkg --build-type ament_cmake
```

### Add python script

```
mkdir scripts

```

- Add execution permission 
- Add `#!` to script

```
chmod +x scripts/py_sub_node.py
```

```python title="add shebang"
#!/usr/bin/env python3
```


---

### Modify CMakeLists.txt

```cmake
install(PROGRAMS
  scripts/py_sub_node.py
  DESTINATION lib/${PROJECT_NAME}
)
```

### package.xml

```xml title="add depend"
  <depend>rclpy</depend>
```


```
my_cpp_py_pkg/
# --> package info, configuration, and compilation
├── CMakeLists.txt
├── package.xml
├── scripts
│   └── py_sub_node.py
├── include
│   └── my_cpp_py_pkg
│       └── cpp_pub_header.hpp
└── src
    └── cpp_pub_node.cpp
```

---

## Reference
- [Create a ROS2 package for Both Python and Cpp Nodes](https://roboticsbackend.com/ros2-package-for-both-python-and-cpp-nodes/)

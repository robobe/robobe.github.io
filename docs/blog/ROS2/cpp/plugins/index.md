---
title: plugins
tags:
    - ros2
    - plugins
    - cpp
---

Pluginlib in ROS 2 is a framework that allows developers to dynamically load and unload plugins at runtime
A plugin is essentially a shared library that provides a specific interface 

**Interfaces**: Specifies the functions that any plugin implementing it must provide. usually implement as abstract class

**Implementations**: The plugin itself is a class that implements the interface. These classes are compiled into **shared** libraries (.so files) that can be loaded dynamically.

**Plugin Manifest**: A manifest file (usually an .xml file) is required to describe the plugin and register it with the pluginlib system. This file specifies the **plugin class** and the **interface** it implements.

**Loading Plugins**: The pluginlib::ClassLoader class is used to load plugins. It locates and loads the appropriate **shared libraries**, ensuring that the plugin adheres to the defined interface.


---

## Demo

### interface

```cpp title="regular_polygon"
#ifndef POLYGON_BASE_REGULAR_POLYGON_HPP
#define POLYGON_BASE_REGULAR_POLYGON_HPP

namespace polygon_base
{
  class RegularPolygon
  {
    public:
      virtual void initialize(double side_length) = 0;
      virtual double area() = 0;
      virtual ~RegularPolygon(){}

    protected:
      RegularPolygon(){}
  };
}  // namespace polygon_base

#endif  // POLYGON_BASE_REGULAR_POLYGON_HPP
```

### CMakeLists
- Export include directory: allowing other packages to properly locate and use the headers of the current package
  
```
ament_export_include_directories(include)
```



---

# Reference
-[Creating and using plugins (C++)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Pluginlib.html)
- [Package Summary](http://wiki.ros.org/pluginlib)
- [ros2 pluginlib](https://github.com/IntelligentSystemsLabUTV/ros2-examples/tree/humble/src/cpp/plugins_demo)
- [ROS 2 Basics - Client Libraries - Creating a Plugin](https://youtu.be/iSXKNMlqmdU)


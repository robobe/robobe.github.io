---
title: ROS2 custom RQT plugin
tags:
    - rqt
    - plugin
---

```
ros2 pkg create rqt_mypkg --build-type ament_python --dependencies rclpy rqt_gui rqt_gui_py 

```

```
src/rqt_mypkg/
├── package.xml
├── plugin.xml
├── resource
│   ├── MyPlugin.ui
│   └── rqt_mypkg
├── rqt_mypkg
│   ├── __init__.py
│   ├── my_module.py
│   └── main.py
├── setup.cfg
└── setup.py
```

## package.xml
- Add line to export section
  
```xml title="package.xml" linenums="1" hl_lines="3"
<export>
    <build_type>ament_python</build_type>
    <rqt_gui plugin="${prefix}/plugin.xml"/>
</export>
```

## plugin.xml
- Place file in package root folder

```xml title="package.xml" linenums="1" hl_lines="2"
<library path="src">
  <class name="My Plugin" type="rqt_mypkg.my_module.MyPlugin" base_class_type="rqt_gui_py::Plugin">
    <description>
        An example Python GUI plugin to create a great user interface.
    </description>
    <qtgui>
      <group>
        <label>Visualization</label>
      </group>
      <!--<group>
          <label>Subgroup</label>
        </group>
        -->
      <label>My first Python Plugin</label>
      <icon type="theme">system-help</icon>
      <statustip>Great user interface to provide real value.</statustip>
    </qtgui>
  </class>
</library>
```

## simple plugin file

---

# Resources
- [rqt_tf_tree](https://github.com/ros-visualization/rqt_tf_tree/tree/humble)
- [rqt python plugin](http://wiki.ros.org/rqt/Tutorials/Writing%20a%20Python%20Plugin)
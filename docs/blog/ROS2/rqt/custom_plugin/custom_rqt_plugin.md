---
tags:
    - rqt
    - plugin
    - python
    - humble
---

# Custom RQT python plugin


- [Create package and init plugin setup (this post)](custom_rqt_plugin.md)
- [Plugin with Qt gui](custom_rqt_plugin_add_gui.md)
- [Build and Deploy](custom_rqt_plugin_test_and_deploy.md)




## Install dependencies
- Install rqt and qt dependencies

```bash
sudo apt install ros-humble-rqt 
sudo apt install ros-humble-qt-gui 
```


## Create package
Create ros python package

```bash
ros2 pkg create rqt_demo --build-type ament_python --dependencies rqt_gui rqt_gui_py
```

## Project structure

```bash
├── package.xml
├── plugin.xml
├── README.md
├── resource
│   ├── rqt_demo
│   └── Demo.ui
├── rqt_demo
│   ├── demo.py
│   ├── __init__.py
├── setup.cfg
└── setup.py
```

## Project setup 
(no widgets just window)

```python title="rqt_demo/demo.py"
from qt_gui.plugin import Plugin

class DemoPlugin(Plugin):
    def __init__(self, context):
        super(Demo, self).__init__(context)

        self._node = context.node

        # Give QObjects reasonable names
        self.setObjectName('RQTDemo')
```

```xml title="plugin.xml"
<library path="src">
    <class name="demo" type="rqt_demo.demo.DemoPlugin" base_class_type="rqt_gui_py::Plugin">
      <description>
        Demo rqt plugin
      </description>
      <qtgui>
        <group>
          <label>Demo</label>
        </group>
        <label>simple demo plugin</label>
        <icon type="theme">system-help</icon>
        <statustip>Simple demo plugin.</statustip>
      </qtgui>
    </class>
  </library>
```

#### setup.py
- Add `plugin.xml` to project data file


```python
data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name, ['plugin.xml']),
],
```

#### package.xml
- Add plugin.xml to package export 

```xml
  <export>
    <build_type>ament_python</build_type>
    <rqt_gui plugin="${prefix}/plugin.xml"/>
  </export>
```

#### Build
```bash
colcon build --symlink-install --merge-install --packages-select rqt_demo
```


#### Run
- Select plugins -> Demo -> simple demo plugin
   



!!! tip rqt arguments
    **-v**:  verbose
    **--force-discover**:      force a rediscover of plugins
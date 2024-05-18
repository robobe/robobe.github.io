---
tags:
    - rqt
    - plugin
    - python
    - humble
---

# RQT python plugin add GUI

TODO: Edit 


- Add `ui` file
- Add Widget file
- Load ui


```xml title="resource/Demo.ui"
<?xml version="1.0" encoding="UTF-8"?>
<ui version="4.0">
 <class>TopicWidget</class>
 <widget class="QWidget" name="TopicWidget">
  <property name="geometry">
   <rect>
    <x>0</x>
    <y>0</y>
    <width>731</width>
    <height>412</height>
   </rect>
  </property>
  <property name="windowTitle">
   <string>Demo plugin</string>
  </property>
  <widget class="QWidget" name="verticalLayoutWidget">
   <property name="geometry">
    <rect>
     <x>170</x>
     <y>60</y>
     <width>160</width>
     <height>251</height>
    </rect>
   </property>
   <layout class="QVBoxLayout" name="verticalLayout">
    
    <item>
     <widget class="QPushButton" name="pushButton">
      <property name="text">
       <string>Demo</string>
      </property>
     </widget>
    </item>
   </layout>
  </widget>
 </widget>
 <resources/>
 <connections/>
</ui>

```

## Widget


```python title="rqt_demo/demo_widget.py"
import os
from ament_index_python import get_resource
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget


class DemoWidget(QWidget):
    def __init__(self, node, plugin):
        super(DemoWidget, self).__init__()

        self._node = node
        self._plugin = plugin

        _, package_path = get_resource('packages', 'rqt_demo')
        ui_file = os.path.join(package_path, 'share', 'rqt_demo', 'resource', 'Demo.ui')
        loadUi(ui_file, self)
        self._node.get_logger().info("loaded")
```

### Plugin
- import widget

```python title="rqt_demo/demo.py"
from qt_gui.plugin import Plugin

from .demo_widget import DemoWidget


class DemoPlugin(Plugin):
    def __init__(self, context):
        super(DemoPlugin, self).__init__(context)

        self._node = context.node

        # Give QObjects reasonable names
        self.setObjectName('RQTDemo')
        self._widget = DemoWidget(self._node, self)
        context.add_widget(self._widget)
```

### setup.py
- Add `ui` file to project datafile

```python 
data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['plugin.xml']),
        ('share/' + package_name + '/resource', ['resource/Demo.ui']),
    ],
```
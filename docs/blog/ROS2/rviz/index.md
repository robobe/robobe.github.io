---
title: ROS2 RVIZ2
tags:
    - rviz
    - rviz2
---


## Tips
### Launch rviz node with config

```python title="setup.py"
(os.path.join('share', package_name, "config"), glob('config/*.rviz'))  
```

```python title="launch.py"
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

PACKAGE_NAME = "py_tutorial_pkg"

def generate_launch_description():
    ld = LaunchDescription()

    rviz_node = Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d' + os.path.join(get_package_share_directory(PACKAGE_NAME), 'config', 'rviz_turtlesim_tf.rviz')]
        )

    ld.add_action(rviz_node)
    return ld
```

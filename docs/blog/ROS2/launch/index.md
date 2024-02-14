---

tags:
    - ros2
    - launch
---
# ROS2 launch 
ROS2 launch file run/launch multiple nodes and allow to add logic to our startup sequence.

Launch file has many features to control the launch sequence 

- Actions
- Event handlers
- substitutions
- conditions

---

## Demo: Minimal launch file

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    sim_node =  Node(
            package='turtlesim',
            namespace='turtlesim1',
            executable='turtlesim_node',
            name='sim'
        )

    ld.add_action(sim_node)
    return ld
```

---

### copy launch folder
#### cmake

```c
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
```

#### python

```python
import os
from glob import glob
from setuptools import setup

package_name = 'py_launch_example'

setup(
    # Other parameters ...
    data_files=[
        # ... Other data files
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.[pxy][yma]*'))
    ]
)
```

---
## Examples and more

- [launch with arguments](launch_with_arguments.md)


---

## Resources
- [ROS2 Launch File Migrator](https://github.com/aws-robotics/ros2-launch-file-migrator/tree/master)
- [rosetta_launch](https://github.com/MetroRobots/rosetta_launch)
- [Design ROS 2 Launch System](https://design.ros2.org/articles/roslaunch.html)
- [ROS2 launch files – All you need to know](https://roboticscasual.com/tutorial-ros2-launch-files-all-you-need-to-know/)
- [Architecture of launch](https://github.com/ros2/launch/blob/humble/launch/doc/source/architecture.rst#id71)
- [launch source github](https://github.com/ros2/launch/tree/humble/launch)
- [Creating a launch file tutorial](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Creating-Launch-Files.html)
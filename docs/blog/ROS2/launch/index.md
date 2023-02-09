---
title: ROS2 launch system
tags:
    - ros2
    - launch
---

# My Tutorials
- [Setup tutorial pkg](setup.md)


- [Run launch file that include other launch files](launch_with_include.md)


---

# Resources
- [ROS2 launch files – All you need to know](https://roboticscasual.com/tutorial-ros2-launch-files-all-you-need-to-know/)
- [Architecture of launch](https://github.com/ros2/launch/blob/humble/launch/doc/source/architecture.rst#id71)
- [launch source github](https://github.com/ros2/launch/tree/humble/launch)
- [Creating a launch file tutorial](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Creating-Launch-Files.html)


---

# Tips
## Minimal launch file

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo

def generate_launch_description():
    ld = LaunchDescription()

    sim_node =  Node(
            package='turtlesim',
            namespace='turtlesim1',
            executable='turtlesim_node',
            name='sim'
        )

    log_launch = LogInfo(msg="---- log launch file ------")

    ld.add_action(log_launch)
    ld.add_action(sim_node)
    return ld
```

---

## copy launch folder
### cmake

```c
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
```

### python

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

### python ext
```python
data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ]

def package_files(data_files, directory_list):
    paths_dict = {}
    for directory in directory_list:
        for (path, directories, filenames) in os.walk(directory):
            for filename in filenames:
                file_path = os.path.join(path, filename)
                install_path = os.path.join('share', package_name, path)
                if install_path in paths_dict.keys():
                    paths_dict[install_path].append(file_path)
                else:
                    paths_dict[install_path] = [file_path]

    for key in paths_dict.keys():
        data_files.append((key, paths_dict[key]))

    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=package_files(data_files, [
        'models/',
        'launch/',
        'worlds/',
        "config",
        "urdf"
    ]),
)
```
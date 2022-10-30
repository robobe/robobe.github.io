---
title: ROS2 launch system
tags:
    - ros2
    - launch
---

# Tutorials
- [Run launch file that include other launch files](launch_with_include.md)


---

# Resources
- [Architecture of launch](https://github.com/ros2/launch/blob/humble/launch/doc/source/architecture.rst#id71)
- [launch source guthub](https://github.com/ros2/launch/tree/humble/launch)
- [Creating a launch file tutorial](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Creating-Launch-Files.html)


---

# tips
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
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*'))
    ]
)
```



---

# Reference
- [Architecture of launch](https://github.com/ros2/launch/blob/humble/launch/doc/source/architecture.rst)
---
tags:
    - gazebo
    - garden
    - gz
---

# Gazebo garden
[gazebosim doc](https://gazebosim.org/docs/garden/install_ubuntu)


## ROS2
[ros_gz](https://github.com/gazebosim/ros_gz)
### install
```
sudo apt install ros-humble-ros-gz
```

### ros-gz packages
- ros_gz: Metapackage which provides all the other packages.
- ros_gz_image: Unidirectional transport bridge for images from Gazebo Transport to ROS using image_transport.
- ros_gz_bridge: Bidirectional transport bridge between Gazebo Transport and ROS.
- ros_gz_sim: Convenient launch files and executables for using Gazebo Sim with ROS.
- ros_gz_sim_demos: Demos using the ROS-Gazebo integration.
- ros_gz_point_cloud: Plugins for publishing point clouds to ROS from Gazebo Sim simulations.

---

## Demo
Basic ROS2 launch file
- Create package ``
- Add launch folder


```python title="simple.launch.py"
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = LaunchDescription()

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': '-r -v 2 empty.sdf'}.items(),
    )

    ld.add_action(gazebo)
    return ld

```

```python title="setup.py"
# Add to data_files

(os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
```

---

### build and run
```bash
# From w.s root folder
colcon build

# source
source install/setup.zsh

# launch
ros2 launch gz_demos simple.launch.py
```

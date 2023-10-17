---
tags:
    - gazebo
    - garden
    - gz
    - simulation
    - ros2
    - launch
    - project
    - template
---

# ROS2 Gazebo Project

A template project integrating ROS 2 and Gazebo simulator.
[base on](https://github.com/gazebosim/ros_gz_project_template)

- `<name>_description`: holds the sdf description of the simulated system and any other assets.
- `<name>_gazebo`: holds gazebo specific code and configurations
- `<name>_application`: holds ros2 specific code and configurations.
- `<name>_bringup`: holds launch files and high level utilities.

!!! warning "Gazebo Garden ROS2 humble"
    ros_gz package binaries not compatible with gazebo garden
    we need to download the package and build it as part of the workspace. check [github ros_gz](https://github.com/gazebosim/ros_gz)


---

## Template 
The template base on ament_cmake packages
I prepared ament_python package, 

!!! warning
    The template project use `dsv` file to config gazebo environment variables `GZ_SIM_RESOURCE_PATH` `GZ_SIM_SYSTEM_PLUGIN_PATH`
    The ament_python not support the `dsv` or i don't find how to use this feature
    I config the environment variable from the launch files
    [more](https://github.com/ros-simulation/gazebo_ros_pkgs/wiki/ROS-2-Migration:-Gazebo-ROS-Paths#env-hooks)

!!! tip "Add copy folder function to setup.py"
    Add helper method to copy `data_files` files and folders recursive.

    ```python
    def package_files(directory_list):
    """helper method to copy data_files recursive
    """
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ]
    paths_dict = {}
    for directory in directory_list:
        for (path, _, filenames) in os.walk(directory):
            for filename in filenames:
                file_path = os.path.join(path, filename)
                install_path = os.path.join('share', package_name, path)
                if install_path in paths_dict:
                    paths_dict[install_path].append(file_path)
                else:
                    paths_dict[install_path] = [file_path]

    for src, dst in paths_dict.items():
        data_files.append((src, dst))

    return data_files
    ```

    ```python title="usage"
    setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=package_files(["models"]),
    ...
    ...
    )
    ```

---

## launch
Basic launch run Gazebo world with model from (`_description`) project

```python title="basic.launch.py" linenums="1" hl_lines="1"
import pathlib
from ament_index_python.packages import get_package_share_directory

import launch
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable


def generate_launch_description():
    ld = launch.LaunchDescription()
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg_project_gazebo = get_package_share_directory("nav_gazebo")
    pkg_project_description = get_package_share_directory("nav_description")

    modules = pathlib.Path(pkg_project_description).joinpath("models").as_posix()
    worlds = pathlib.Path(pkg_project_gazebo).joinpath("worlds").as_posix()

    resources = ":".join([modules, worlds])

    gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[EnvironmentVariable("GZ_SIM_RESOURCE_PATH"), resources],
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            pathlib.Path(pkg_ros_gz_sim)
            .joinpath("launch", "gz_sim.launch.py")
            .as_posix()
        ),
        launch_arguments={
            "gz_args": "-v4 -r empty.sdf"
        }.items(),
    )

    ld.add_action(gz_resource_path)
    ld.add_action(gz_sim)
    return ld

```
     
     
     

     
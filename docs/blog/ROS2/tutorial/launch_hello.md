---
title: Part2 - Launch file
description: Basic ROS2 launch file
date: "2022-04-05"
banner: ../ros2.png
tags:
    - ros2
    - launch
    - 101
---

## launch source code
```python title="hello.launch.py"
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    talker_node = Node(
        package="basic",
        executable="simple_pub",
    )

    listener_node = Node(
        package="basic",
        executable="simple_sub"
    )

    ld.add_action(talker_node)
    ld.add_action(listener_node)

    return ld
```

!!! Note
    Your launch file must contain this function: generate_launch_description(), and must return a LaunchDescription object.

---
    
## setup.py
- Add entry to `data_files` copy the launch folder to package install folder

```python title="setup.py"  linenums="1" hl_lines="4"
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name), glob("launch/*.launch.py"))
    ],
```

!!! Note
    copy launch files using CMakeLists.txt
    ```cmake
    ...

        install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}
        )
    ```
---

## usage
```bash title="terminal1"
ros2 launch basic hello.launch.py 
# Result
[INFO] [launch]: All log files can be found below /home/user/.ros/log/2022-04-06-08-52-59-435522-lap2-73549
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [simple_pub-1]: process started with pid [73551]
[INFO] [simple_sub-2]: process started with pid [73553]
[simple_pub-1] [INFO] [1649224380.693329543] [minimal_publisher]: Publishing: "pub simple: 0"
[simple_sub-2] [INFO] [1649224380.693398516] [minimal_subscriber]: I heard: pub simple: 0
[simple_pub-1] [INFO] [1649224381.683970139] [minimal_publisher]: Publishing: "pub simple: 1"
[simple_sub-2] [INFO] [1649224381.684546630] [minimal_subscriber]: I heard: pub simple: 1

```


---

# References
- [ROS2 Launch File Example – How to Start All Your Nodes at Once](https://roboticsbackend.com/ros2-launch-file-example/)
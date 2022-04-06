---
title: Part4 - Launch with arguments
description: Launch file with arguments and config
date: "2022-04-05"
banner: ../ros2.png
tags:
    - ros2
    - param
    - parameter
    - launch
    - 101
---

## 

- **LaunchConfiguration** is local to the launch file and scoped.
- **DeclareLaunchArgument** allows you to expose the argument outside of your launch file. Allowing them to be listed, set, or marked as required when a user launches it from the command line (using ros2 launch) or when including it from another launch file (using IncludeLaunchDescription).

```python title="basic_demo_args.launch.py"
import launch

def generate_launch_description():
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument('msg', default_value='hello world'),
        launch.actions.DeclareLaunchArgument('other'),
        launch.actions.LogInfo(msg=launch.substitutions.LaunchConfiguration('msg')),
        launch.actions.LogInfo(msg=launch.substitutions.LaunchConfiguration('other')),
    ])
```

```bash
ros2 launch basic demo_args.launch.py other:="other message"
# Result
[INFO] [launch]: All log files can be found below /home/user/.ros/log/2022-04-06-11-27-22-383934-lap2-90291
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [launch.user]: hello world
[INFO] [launch.user]: other message
```

!!! Warning
    `other` argument is mandatory because has no `default value`

#### cli
```bash
ros2 launch -s basic demo_args.launch.py
# Result
Arguments (pass arguments as '<name>:=<value>'):

    'msg':
        no description given
        (default: 'hello world')

    'other':
        no description given
```

## demo: launch with arguments

```python title="simple_param_args.launch.py" linenums="1" hl_lines="3"

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
        
    my_str_arg = DeclareLaunchArgument("my_str", default_value="world")
    my_str = LaunchConfiguration("my_str")    
    node=Node(
        name="simple_params",
        package = 'basic',
        executable = 'simple_param',
        parameters = [
            {"my_str": my_str},
            {"my_int": 1000},
            {"my_double_array": [1.0, 10.0]}
        ]
    )
    ld.add_action(my_str_arg)
    ld.add_action(node)
    return ld
```

### Usage
```bash linenums="1" hl_lines="1 6"
ros2 launch basic simple_param_args.launch.py my_str:=worlddddddddddddd
# result
[INFO] [launch]: All log files can be found below /home/user/.ros/log/2022-04-06-14-10-00-346206-lap2-93999
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [simple_param-1]: process started with pid [94001]
[simple_param-1] [INFO] [1649243402.580006387] [simple_params]: Hello worlddddddddddddd! with int data: 1000
[simple_param-1] [INFO] [1649243402.580249948] [simple_params]: array('d', [1.0, 10.0])

```
---

# References
- [what is different between DeclareLaunchArgument and LaunchConfiguration](https://answers.ros.org/question/322874/ros2-what-is-different-between-declarelaunchargument-and-launchconfiguration/)
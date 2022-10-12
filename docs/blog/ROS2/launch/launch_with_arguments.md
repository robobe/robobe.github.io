---
title: launch with arguments
tags:
    - launch
    - argument
    - ros2
---


```python
from launch import LaunchDescription
from launch.actions import LogInfo, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ld = LaunchDescription()
    arg_cmd = DeclareLaunchArgument(
        "msg", default_value="hello world", description="msg simple description"
    )

    ld.add_action(arg_cmd)
    ld.add_action(LogInfo(msg=LaunchConfiguration("msg")))
    return ld
```

## usage

```bash title="show arguments"
# -s, --show-args, --show-arguments                        Show arguments that may be given to the launch file.
ros2 launch launch_tutorials args.launch.py -s

Arguments (pass arguments as '<name>:=<value>'):

    'msg':
        msg simple description
        (default: 'hello world')

```

```bash title="default argument"
ros2 launch cpp_tutrial_pkg basic_launch.launch.py
[INFO] [launch]: All log files can be found below /home/user/.ros/log/2022-10-12-07-20-56-408482-lap2-3896306
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [launch.user]: hello world

```

```bash title="with argument"
ros2 launch cpp_tutrial_pkg basic_launch.launch.py msg:="hello launch"
...
[INFO] [launch.user]: hello launch

```
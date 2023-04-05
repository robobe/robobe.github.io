---
tags:
    - launch
    - argument
    - ros2
    - DeclareLaunchArgument
    - LaunchConfiguration
---
# launch with arguments
Control launch with argument from outside  


**DeclareLaunchArgument** is used to define the launch argument that can be passed from the above launch file or from the console.

**LaunchConfiguration** substitutions allow us to acquire the value of the launch argument in any part of the launch description.

```python
from launch import LaunchDescription
from launch.actions import LogInfo, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ld = LaunchDescription()
    arg1 = LaunchConfiguration("arg1")
    arg1_arg = DeclareLaunchArgument(
        name = "arg1",
        default_value="arg1 default value", 
        description="arg1 description"
    )

    log_action = LogInfo(msg=arg1)
    ld.add_action(arg1_arg)
    ld.add_action(log_action)
    return ld

```

## usage
### launch without arguments
```bash 
ros2 launch pkg_launch_tutorial minimal_arg.launch.py 

...
[INFO] [launch.user]: arg1 default value

```
### check for arguments
```bash
ros2 launch pkg_launch_tutorial minimal_arg.launch.py -s
Arguments (pass arguments as '<name>:=<value>'):

    'arg1':
        arg1 description
        (default: 'arg1 default value')

```
### launch with arguments
```bash title="show arguments"
ros2 launch pkg_launch_tutorial minimal_arg.launch.py arg1:="new arg from cli"
...
[INFO] [launch.user]: new arg from cli


```
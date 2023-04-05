---
tags:
    - launch
    - argument
    - ros2
    - OpaqueFunction
---

# OpaqueFunction
Action that executes a Python function.


## demo
- Execute shell script using `ExecuteProcess` action
- Using `OpaqueFunction` to build shell script arguments
- Convert `LaunchConfiguration` to string using `context.perform_substitution`

### shell script to execute
```sh title="hello.zsh"
#!/bin/zsh

echo $1 > /tmp/1
```

### launch file
```python title="opa" linenums="1" hl_lines="10 18 32"
from launch import LaunchDescription, LaunchContext
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction

SCRIPT_PATH = "/home/user/ros2_ws/src/pkg_launch_tutorial/scripts/hello.zsh"
ARG1="arg1"

def func_demo(context: LaunchContext, arg1: LaunchConfiguration):
    value = context.perform_substitution(arg1)
    
    if not value:
        value = "default value"
    run_script = ExecuteProcess(
        cmd=[[SCRIPT_PATH, " ", f"'{value}'"]],
        shell=True
    )
    return [run_script]



def generate_launch_description():
    ld = LaunchDescription()
    arg1 = LaunchConfiguration(ARG1)
    arg1_arg = DeclareLaunchArgument(
        ARG1, default_value="", description="arg1"
    )
    
    func_action = OpaqueFunction(function=func_demo, args=[arg1])
    
    ld.add_action(arg1_arg)
    ld.add_action(func_action)
    
    return ld

```
---
title: Pass launch argument to control XACRO logic
tags:
    - xacro
    - tutorial
    - launch
---

# LAB
- Control xacro from outside with args
- Bind xacro args to launch args

## xacro

 - Simple xacro file with `arg` and `if`
  
```xml
<?xml version="1.0"?>
<robot name="" xmlns:xacro="http://www.ros.org/wiki/xacro">
    <xacro:arg name="with_gripper" default="true" />
    <xacro:if value="$(arg with_gripper)">
        <!--with gripper-->
    </xacro:if>
</robot>
```

## launch
- using `OpaqueFunction` function to get `LaunchContext` for substitute


```python title="test_xacro.launch.py" linenums="1" hl_lines="23"
import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
import xacro

PACKAGE = "basic_mobile_robot"
SDF = "arg.sdf.xacro"

def generate_launch_description():
    pkg = get_package_share_directory(PACKAGE)
    robot_description_path = os.path.join(pkg, "demos", "xacros")
    robot_description_file = os.path.join(robot_description_path, SDF)
    robot_description_file_out = os.path.join(robot_description_path, "arg.sdf")

    with_gripper_arg = DeclareLaunchArgument(
        'with_gripper',
        default_value="true",
        description="use with_gripper"
    )

    def render_xacro(context: LaunchContext, grip_arg):
        grip_arg_str = context.perform_substitution(grip_arg)
    
        doc = xacro.parse(open(robot_description_file))
        xacro.process_doc(doc, mappings={"with_gripper": grip_arg_str})
        out = xacro.open_output(robot_description_file_out)
        out.write(doc.toprettyxml(indent='  '))

    func = OpaqueFunction(function=render_xacro, args=[LaunchConfiguration('with_gripper')])

    ld = LaunchDescription()
    ld.add_action(with_gripper_arg)
    ld.add_action(func)
    return ld
    
```

## usage

```bash
ros2 launch basic_mobile_robot test_xacro.launch.py with_gripper:="true"
```
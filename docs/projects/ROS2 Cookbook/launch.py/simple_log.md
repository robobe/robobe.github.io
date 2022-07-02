---
title: Simple launch file with log
tags:
    - log
    - launch
---

```python
import launch
from launch import logging

log = logging.get_logger("simple")

log.error("error log line")
log.info("info log line")

def generate_launch_description():
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument('msg', default_value='hello world'),
        launch.actions.DeclareLaunchArgument('other'),
        launch.actions.LogInfo(msg=launch.substitutions.LaunchConfiguration('msg')),
        launch.actions.LogInfo(msg=launch.substitutions.LaunchConfiguration('other', default="aa")),
    ])
```
---
tags:
    - ros2
    - vscode
---

# ROS2 and VSCode
[ROS 2 and VSCode](https://picknik.ai/vscode/docker/ros2/2024/01/23/ROS2-and-VSCode.html)


## Python

```json
{
  "python.autoComplete.extraPaths": [
    "/opt/ros/humble/lib/python3.10/site-packages",
    "/opt/ros/humble/local/lib/python3.10/dist-packages",
    "/my_project/build/package1",
    "/my_project/build/package2"
  ],
  "python.analysis.extraPaths": [
    "/opt/ros/humble/lib/python3.10/site-packages",
    "/opt/ros/humble/local/lib/python3.10/dist-packages",
    "/my_project/build/package1",
    "/my_project/build/package2"
  ]
}
```
---
title: rosdep tutorial
tags:
    - rosdep
---

rosdep is ROS’s dependency management utility that can work with ROS packages and external libraries.


## usage
rosdep will check for `package.xml` files in its path or for a specific package and find the rosdep keys stored within

```
rosdep install --simulate --from-paths ~/workspaces/project/src --ignore-src
```

- `--form-path` package.xml location
---

# Reference
- [Managing Dependencies with rosdep](https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html)
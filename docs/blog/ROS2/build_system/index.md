---
tags:
    - ros2
    - build system
    - colcon
---

# Colcon

Manage builds with colcon
- Dependency management between packages
- Calling the build tool for each package
- Environment settings at build time/run time


## colcon command and package.xml
### colcon command overview
- A build system that makes it easy to build and test multiple packages
    - Managing dependencies between packages
    - Call the build tool and build the package

1.1 colcon verb
- build: Build the package
- test, test_result: Test the package
- info, list, graph: Get package information
- mixins: managing mixins
- metadata: managing metadata

1.2 colcon command line arguments
- Command line arguments are different for each verb
- Commonly used command line arguments
    - directory management
    - package selection
    - Passing arguments to build tools/test tools
    - others

colcon command line argument example
- directory management
    - `colcon build --install-base`: Change the installation directory
  
page 24
### command line argument
### log output
### colcon mixin
### package.xml
### package search, dependency analysis

## ament_cmake advanced

## Reference
- [understanding of the ROS2 build system](https://speakerdeck.com/fixstars/autonomous-driving-realization-by-ros2?slide=64)
- [fixstars ros2-seminar ament sample](https://github.com/fixstars/ros2-seminar-ament-sample)
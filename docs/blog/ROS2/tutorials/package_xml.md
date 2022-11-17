---
title: Package.xml
tags:
    - ros2
---


## depend

A package’s package.xml file contains a set of dependencies. The dependencies in this file are generally referred to as “rosdep keys”. These are represented in the tags <depend>, <test_depend>, <exec_depend>, <build_depend>, and <build_export_depend>. They specify in what situation each of the dependencies are required in.

- For dependencies only used in testing the code (e.g. gtest), use **test_depend**.
- For dependencies only used in building the code, use **build_depend**.
- For dependencies needed by headers the code exports, use **build_export_depend**.
- For dependencies only used when running the code, use **exec_depend**.
- For mixed purposes, use **depend**, which covers **build**, **export**, and **execution** time dependencies.

These dependencies are manually populated in the package.xml file by the package’s creators and should be an exhaustive list of any non-builtin libraries and packages it requires.

---

# Reference
- [Managing Dependencies with rosdep](https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html)
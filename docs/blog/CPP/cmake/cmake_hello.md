---
title: cmake 101
description: cmake 101
date: "2022-06-04"
banner: images/cmake.jpeg
tags:
    - cmake
    - 101
---

build system is a software tools that is used to facilitate the build process

build is the process of “translating” source code files into executable binary code files


## cmake hello

```cpp
cmake_minimum_required(VERSION 3.11)
project(HelloWorld)
add_executable(hello_world hello_world.c)
```

## Add compiler option

```c
add_compile_options(-Wall -Wextra -Wpedantic)
```

---

# Reference
- [CMake: Best Practices](https://indico.jlab.org/event/420/contributions/7961/attachments/6507/8734/CMakeSandCroundtable.slides.pdf)
- [More Modern CMake](https://hsf-training.github.io/hsf-training-cmake-webpage/index.html)
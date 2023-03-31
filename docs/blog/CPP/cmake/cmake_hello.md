---
date: "2022-06-04"
banner: images/cmake.jpeg
tags:
    - cmake
    - 101
---

# CMAKE beginner step
`build system` is a set of programs that build a software codebase -> producing a final products from source code: executable files, share object (.so) files, static libraries (.a)
for example: [Ninja](https://ninja-build.org/), [Make](https://www.gnu.org/software/make/)

cma

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
---
title: cross compiler hello
description: Install ARM cross-compiler on ubuntu machine and use it's for hello world 
date: "2022-03-06"
banner: images/nano.png
tags:
    - cross-compiler
---

A cross compiler is a compiler capable of creating executable code for a platform other than the one on which the compiler is running (wikipedia)

## install cross compiler
```bash title="install crosscompiler tools"
sudo apt install crossbuild-essential-arm64
```

## toolchain setting file
```cmake title="aarch64-linux-gnu-toolchain.cmake"
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR "aarch64")
set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)
set(CMAKE_CUDA_COMPILER nvcc)
set(CMAKE_CUDA_HOST_COMPILER aarch64-linux-gnu-gcc)
set(CMAKE_FIND_ROOT_PATH "/usr/aarch64-linux-gnu")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
```

## demo

```bash
├── aarch64-linux-gnu-toolchain.cmake
├── build
├── CMakeLists.txt
└── src
    └── hello.cpp
```

### code

```cpp
#include <iostream>

int main(){
    std::cout << "hello cross compiler" << std::endl;
    return 0;
}
```

```cmake
cmake_minimum_required(VERSION 3.15)
# Toolchain settings
set(CMAKE_TOOLCHAIN_FILE aarch64-linux-gnu-toolchain.cmake)

project(CrossCompiler_demo)

add_executable(hello_cc src/hello.cpp)
```

!!! warning
    It is crucial to set the value of **CMAKE_TOOLCHAIN_FILE** before `project()` is invoked, because `project()` triggers **toolchain detection and verification**.
     
---

## cmake gui

- using cmake gui
- set cross compiler settings from file
- 
![](images/2022-06-03-16-33-44.png)

![](images/2022-06-03-16-35-01.png)


## make, check, run

### make

- Run `make` from `build` folder
- Check executable arch with `file` command
- Run with qemu or Copy to embedded device

### check
```bash
cd build
# Result
file hello_cc 
hello_cc: ELF 64-bit LSB shared object, ARM aarch64, version 1 (SYSV), dynamically linked, interpreter /lib/ld-linux-aarch64.so.1, BuildID[sha1]=5425e3fd790ba1a6a07c4963f0606a58edf53aa7, for GNU/Linux 3.7.0, not stripped
```

### run
Run ARM binary using `qemu`

```
sudo apt-get install qemu-user-static
```

```bash
# from build folder
qemu-arm-static -L /usr/arm-linux-gnueabihf ./hello_cc
```

---

# Ref
- [How to cross-compile for embedded with CMake like a champ](https://kubasejdak.com/how-to-cross-compile-for-embedded-with-cmake-like-a-champ)
- [How to Cross Compile OpenCV and MXNET for NVIDIA Jetson ](https://medium.com/trueface-ai/how-to-cross-compile-opencv-and-mxnet-for-nvidia-jetson-aarch64-cuda-99d467958bce)
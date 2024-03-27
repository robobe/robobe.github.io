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

CMAKE is a **scripting** language that **generated** instruction to build system like Make or Ninja

---

## cmake hello
- Project `CMakeLists.txt" file
- Build folder

```cpp title="CMakeLists.txt"
cmake_minimum_required(VERSION 3.11)
project(HelloWorld  VERSION 0.0.1
                    DESCRIPTION "hello cpp project")
                    LANGUAGES CXX
add_executable(hello_world hello_world.c)
```

---

```bash
# from project root
#-S : source folder
#-B : build folder
cmake -S . -B build
cmake --build build -j 8
```

### Pipeline
- configure
  - `CMakecache.txt`
- Generate
  - `Makefile`


---

## Variables and Cache
- Local scope variable (not save in (CMakeCache))
- Cached variable

```bash title="local  scope"
# Local variable hides any cached variable with the same name
set(VAR "value")  # Does not get copied to cache!
```

```bash title="local  scope"
# IMPORTANT: Cached variable only set if not found already in cache
set(VAR "value" CACHE STRING "Description")
```

```bash title="first run"
########################
# EXTERNAL cache entries
########################

//cc cachce
CC:STRING=cahced
```

!!! warning "CACHE variable"
    if variable already exist in cached it's not update and cmake use
    the value from cache

    ```
    set(CC "second run" CACHE STRING "Description")
    ```

    Add `FORCE` to override cached
     
    ```
    set(CC "second run" CACHE STRING "Description" FORCE)
    ```

### Set variable

```bash
cmake -D FOO="foo" -S . -B build
```

---

## Messages

```make
message(STATUS "My awesome message")
message("Always print this")
```

---

## Set cpp standard flag

```c
set(CMAKE_CXX_STANDARD 17)
```

## Add compiler option

```c
add_compile_options(-Wall -Wextra -Wpedantic)
```

---

# Reference
- [CMake - the essential package](https://www.youtube.com/watch?v=UH6F6ypdYbw&list=PLwhKb0RIaIS1sJkejUmWj-0lk7v_xgCuT&index=17)
- [CMake: Best Practices](https://indico.jlab.org/event/420/contributions/7961/attachments/6507/8734/CMakeSandCroundtable.slides.pdf)
- [More Modern CMake](https://hsf-training.github.io/hsf-training-cmake-webpage/index.html)
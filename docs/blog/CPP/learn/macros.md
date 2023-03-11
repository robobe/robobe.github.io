---
title: C++ MACROS learn by doing
tags:
    - cpp
    - c++
---
Macros are section of text that replace by the compiler preprocessor stage, it's very useful in DEBUGGING for example

## Demo
Control with macro to use 

```cpp title="main.cpp"
#include <iostream>

#if MY_DEBUG == 1
    #define LOG(x) std::cout << x << std::endl;
#else
    #define LOG(x)
#endif

int main() {
    LOG("HELLO")
}
```

### build and run
```bash title="terminal"
g++ main.cpp -o main -DMY_DEBUG=0
./main
# print nothing
 g++ main.cpp -o main -DMY_DEBUG=1
./main
HELLO
```

### CMakeLists.txt
```c
cmake_minimum_required(VERSION 3.1)
project(demo_macros)

add_definitions(-DMY_DEBUG=0)
add_executable(main main.cpp)
```

### CMakeLists with options
```c
cmake_minimum_required(VERSION 3.1)
project(demo_macros)

option(USE_DEBUG "Enter debug mode" OFF)
if (USE_DEBUG)
    add_definitions(-DMY_DEBUG=1)
endif()

add_executable(main main.cpp)
```

#### usage
```bash title="options" linenums="1" hl_lines="6"
cmake -LA ..
#
...
CMAKE_STRIP:FILEPATH=/usr/bin/strip
CMAKE_VERBOSE_MAKEFILE:BOOL=FALSE
USE_DEBUG:BOOL=OFF

#
cmake -DUSE_DEBUG=ON ..
```

---

# References
- [Macros in c++](https://youtu.be/j3mYki1SrKE?list=PLlrATfBNZ98dudnM48yfGUldqGD0S4FFb)
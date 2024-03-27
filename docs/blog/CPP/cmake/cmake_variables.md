---
tags:
    - cmake
    - cpp
    - variables
---


## Use CMAKE variable in cpp code

```c title="CMakeLists.txt"
cmake_minimum_required(VERSION 3.2.0)
project(cpp_tutorial VERSION 0.1.0
                    DESCRIPTION "hello cmake"
                    LANGUAGES CXX
)

SET (CUSTOM "custom")
ADD_DEFINITIONS( -D_CMAKE_MESSAGE="${CUSTOM}" )

add_executable(hello src/hello.cpp)
```

```cpp title="src/hello.cpp"
#include <iostream>

#define CMAKE_MESSAGE _CMAKE_MESSAGE

auto main() -> int
{
    std::cout << "message from cmake variable: " << CMAKE_MESSAGE << std::endl;
    return EXIT_SUCCESS;
}
```

---

## Using header file and substitution

```
├── CMakeLists.txt
└── src
    ├── hello.cpp
    ├── hello.h
    └── hello.h.in
```

```cpp title="hello.h.in"
#ifndef VERSION_H_IN
#define VERSION_H_IN

#define CUSTOM_MESSAGE "@CUSTOM@"


#endif // VERSION_H_IN
```

```cpp title="hello.cpp"
#include <iostream>
#include <hello.h>


auto main() -> int
{
    std::cout << "message from cmake header variable: " << CUSTOM_MESSAGE << std::endl;
    return EXIT_SUCCESS;
}
```

```c title="CMakeLists.txt"
cmake_minimum_required(VERSION 3.2.0)
project(cpp_tutorial VERSION 0.1.0
                    DESCRIPTION "hello cmake"
                    LANGUAGES CXX
)

SET (CUSTOM "qqq" CACHE STRING "custom description")
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)
configure_file (src/hello.h.in ${CMAKE_CURRENT_SOURCE_DIR}/src/hello.h @ONLY)
add_executable(hello src/hello.cpp)
```

!!! note "configure_file"
    Convert `hello.h.in` file to `hello.h` and run substitution on every **@VAR_VALUE@** variable
     

```bash usage
# from project root
cmake -D CUSTOM="aaa" -S . -B build

# or
cd build
cmake -D CUSTOM="bbb" ..
make
```
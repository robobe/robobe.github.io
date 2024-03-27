---
tags:
    - cmake
    - cpp
    - demo
    - 
---

- Create executable
- Use Headers
- Set install folder



```
├── bin
├── CMakeLists.txt
├── include
│   └── utils.hpp
└── src
    ├── CMakeLists.txt
    ├── hello.cpp
    └── utils.cpp
```

```c title="root CMakeLists.txt"
cmake_minimum_required(VERSION 3.2.0)
project(cpp_tutorial VERSION 0.1.0
                    DESCRIPTION "hello cmake"
                    LANGUAGES CXX
)

add_subdirectory(src)
```

```cpp title="include/utils.hpp"
#pragma once
namespace cmake_tutorial{
    namespace utils{
        int add(int x, int y);
    }
}

```cpp title="src/utils.cpp"
#include "utils.hpp"

namespace cmake_tutorial
{
    namespace utils
    {
        int add(int x, int y)
        {
            return x + y;
        }
    }
}
```

```cpp title="src/hello.cpp"
#include <iostream>
#include <utils.hpp>


auto main() -> int
{
    auto result = cmake_tutorial::utils::add(1,2);
    std::cout << "hello:" << result << std::endl;
    return EXIT_SUCCESS;
}
```

```c title="src/CMakeLists.txt"
set(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR} CACHE PATH "install prefix" FORCE)
add_executable(hello hello.cpp utils.cpp)
target_include_directories(hello PRIVATE ${PROJECT_SOURCE_DIR}/include)

install(TARGETS
    hello
    DESTINATION
    bin )
```

- Set `CMAKE_INSTALL_PREFIX` force override **CACHE** value
- Add include folder to `hello` target
- Add install target

```bash
cmake -S. -B build
```

```bash
cmake --build build
```

```bash
cmake --install build
```

```bash title="override prefix using cli"
cmake --install build --prefix /tmp
```
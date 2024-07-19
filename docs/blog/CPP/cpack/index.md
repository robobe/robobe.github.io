---
tags:
    - cpack
    - cmake
    - deb
    - package
    - dpkg
---

# CPack

CPack is responsible for making packages. from cmake output
CPack **pack** all the output from cmake `install()`  statement


## Simple demo
pack simple hello cpp program to deb

```title="project tree"
.
├── build
├── cmake
│   └── Packing.cmake
├── CMakeLists.txt
└── src
    └── hello.cpp
```

```cpp title="src/hello.cpp"
#include <iostream>

int main(int argc, char const *argv[])
{
    std::cout << "hello cpack" << std::endl;
    return 0;
}
```

```bash title="CMakeLists.txt"
cmake_minimum_required(VERSION 3.20)
project(cpack_tutorial)

add_executable(hello src/hello.cpp)

install(TARGETS hello DESTINATION bin)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(Packing)
```

```bash title="cmake.Packing.cmake"
set(CPACK_PACKAGE_NAME ${PROJECT_NAME}
    CACHE STRING "The resulting package name"
)

set(CPACK_PACKAGE_CONTACT "YOUR@E-MAIL.net")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "YOUR NAME")

# Set the CPack generator to DEB
set(CPACK_GENERATOR "DEB")

include(CPack)
```

### usage
from build folder run
- cmake ..
- make
- cpack


### deb analysis

!!! note dpkg
    -c : List contents of the deb package
    -I : Show information about a package  (show package dependencies)
    -e : Extract control information `dpkg -e <deb file> <folder name>`
---

## Reference
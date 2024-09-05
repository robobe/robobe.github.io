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

## Demo
Pack simple hello world binary as deb

```title="project"
├── CMakeLists.txt
├── _packages
├── README.md
├── scripts
│   ├── postinst
│   └── preinst
└── src
    └── main.cpp
```
```C title=main
#include <iostream>

using namespace std;

int main(int argc, char** argv) {
    cout << "hello cmake" << endl;
}
```

```bash title="postinst"
#!/bin/bash

echo "post" >> /tmp/aaaa
```

```bash title="preinst"
#!/bin/bash

echo "pre" >> /tmp/aaaa
```

```c
cmake_minimum_required(VERSION 3.9)

project(cmake_cpp_template)
set(CMAKE_CXX_STANDARD 17)
#executable
add_executable(app src/main.cpp)
install(TARGETS app DESTINATION bin)
#cpack
set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA ${CMAKE_CURRENT_SOURCE_DIR}/scripts/postinst;${CMAKE_CURRENT_SOURCE_DIR}/scripts/preinst)
set(CPACK_OUTPUT_FILE_PREFIX "${CMAKE_SOURCE_DIR}/_packages")
set(CPACK_GENERATOR DEB)
set(CPACK_DEBIAN_FILE_NAME DEB-DEFAULT)
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "robobe")
set(CPACK_PACKAGE_VERSION_MAJOR 0)
set(CPACK_PACKAGE_VERSION_MINOR 1)
set(CPACK_PACKAGE_VERSION_PATCH 0)
include(CPack)
```

### usage
```
mkdir build
cd build
cmake ..
make
cpack
```

### Analysis

#### info
!!! note "control"
    in the control section we note the `postinst` and `preint` scripts


     
```bash
dpkg -I _packages/cmake_cpp_template_0.1.0_amd64.deb 
#
 new Debian package, version 2.0.
 size 3478 bytes: control archive=379 bytes.
     186 bytes,     9 lines      control              
      46 bytes,     1 lines      md5sums              
      37 bytes,     2 lines      postinst             #!/bin/bash
      40 bytes,     2 lines      preinst              #!/bin/bash
 Architecture: amd64
 Description: cmake_cpp_template built using CMake
 Maintainer: robobe
 Package: cmake_cpp_template
 Priority: optional
 Section: devel
 Version: 0.1.0
 Installed-Size: 25
```

#### Content

```bash
dpkg -c _packages/cmake_cpp_template_0.1.0_amd64.deb 
#
drwxrwxr-x root/root         0 2024-09-05 07:31 ./usr/
drwxrwxr-x root/root         0 2024-09-05 07:31 ./usr/bin/
-rwxr-xr-x root/root     16528 2024-09-05 07:31 ./usr/bin/app
```
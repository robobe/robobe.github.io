---
tags:
    - eigen
    - cpp
    - linear algebra
---

# Eigen


## install

```bash
sudo apt install libeigen3-dev
```

## Hello world
Simple usage of matrix

- ubuntu 22.04
- vscode
  
```cpp title="hello.cpp"
#include <iostream>
#include <Eigen/Dense>
 

int main()
{
  Eigen::Matrix<float,3,3> matrixA;
  matrixA.setZero();
  std::cout << matrixA << std::endl;
}
```

### Build
#### compiler
```
g++ -std=c++17 -I/usr/include/eigen3 hello.cpp -o hello
```
#### CMake
```c title="CMakeLists.txt"
cmake_minimum_required (VERSION 3.0)
project (myproject)
 
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

add_executable (hello hello.cpp)
target_link_libraries (hello Eigen3::Eigen)
```

### Config vscode
```json title="c_cpp_properties.json"
{
    "configurations": [
        {
            "name": "Linux",
            "includePath": [
                "${workspaceFolder}/**",
                "/usr/include/eigen3/**"
            ],
            "defines": [],
            "compilerPath": "/usr/bin/clang",
            "cStandard": "c17",
            "cppStandard": "c++14",
            "intelliSenseMode": "linux-clang-x64"
        }
    ],
    "version": 4
}
```

---
title: ROS2 basic gtest
tags:
    - gtest
    - ros2
---

# LAB
- Create GTest for ROS2 package
- Create ROS2 package as Library

## Project

```bash
my_cpp_library/
├── CMakeLists.txt
├── include
│   └── my_cpp_library
│       └── library_header.h
├── package.xml
├── src
│   ├── main.cpp
│   └── my_cpp_library.cpp
└── test
    ├── CMakeLists.txt
    ├── demo_test.cpp
    └── main.cpp
```

### Library
#### include folder
```cpp title="my_cpp_library/include/my_cpp_library/library_header.h"
#pragma once

namespace demo{
    int add(int a, int b);
}
```

#### src folder
```cpp title="my_cpp_library/src/my_cpp_library.cpp"
#include "library_header.h"

namespace demo{
    int add(int a, int b){
        return a+b;
    }
}
```

```cpp title="my_cpp_library/src/main.cpp"
#include <iostream>
#include "library_header.h"

int main()
{
    std::cout << demo::add(1, 2) << std::endl;
    return 0;
}
```

#### test folder

```cpp title="my_cpp_library/test/main.cpp"
#include "gtest/gtest.h"

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
```

```cpp title="demo_test.cpp"
#include "gtest/gtest.h"
#include "library_header.h"

TEST(MyFirstTestSuite, DemoTest)
{
    EXPECT_TRUE(true);
}

TEST(MyFirstTestSuite, AddTest)
{
    auto result = demo::add(1, 2);
    EXPECT_TRUE(result == 3);
}
```

```c title="test/CMakeLists.txt"
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  set(TESTFILES
      main.cpp
      demo_test.cpp
      )
  # Add gtest executables
  ament_add_gtest(${PROJECT_NAME}_test ${TESTFILES})
  target_link_libraries(${PROJECT_NAME}_test my_lib)

  install(TARGETS
        ${PROJECT_NAME}_test
        DESTINATION lib/${PROJECT_NAME})
  endif()
```

---

## package

```c title="CMakeLists.txt"
cmake_minimum_required(VERSION 3.8)
project(my_cpp_library)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

include_directories(include/my_cpp_library)

# Create library
add_library(my_lib src/my_cpp_library.cpp)
# main
add_executable(main src/main.cpp)
target_link_libraries(main my_lib)

install(TARGETS 
  main
  DESTINATION lib/${PROJECT_NAME}/
)

install(
  TARGETS my_lib
  EXPORT my_lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

add_subdirectory(test)

ament_package()
```

---

## usage
- build 
- test
- test-result

### build
```
colcon build --package-select my_cpp_library
```

### test

```bash
# Run test
colcon test --packages-select my_cpp_library

# Run test with verbose output
colcon test --packages-select my_cpp_library --event-handler=console_direct+

```

### test-result
Show only result summery

```bash
colcon test-result --test-result-base build/my_cpp_library
```
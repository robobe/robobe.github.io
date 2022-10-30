---
title: GTest with cmake and VSCode
tags:
    - cpp
    - gtest
    - cmake
---

# LAB objective
- Build cpp project with library using cmake
- Create testing with GTest
- Config VSCode


## install


```bash
sudo apt install libgtest-dev
```

```
├── app
│   ├── CMakeLists.txt
│   └── src
│       └── main.cpp
├── helper
│   ├── CMakeLists.txt
│   ├── include
│   │   └── helper
│   └── src
│       └── helper.cpp
├── tests
│    ├── CMakeLists.txt
│    └── test_helper.cpp
│
└── CMakeLists.txt
```

---

## lib
- header file (definition)
- cpp file (declaration)
- cmake file 

```cpp title="include/helper/helper.h"
#pragma once

namespace helper{
    int add(int x, int y);
}
```

```cpp title="src/helper.cpp"
#include "helper.h"

namespace helper
{
    int add(int x, int y)
    {
        return x + y;
    }
}
```

```c title="CMakeLists.txt"
set (SOURCES
    src/helper.cpp
)
add_library(helper ${SOURCES})
target_include_directories(helper PUBLIC include/helper)

```

---

# app
- cpp file
- cmake file


```cpp title="app/src/main.cpp"
#include <iostream>
#include <helper.h>

using namespace std;

int main(int argc, char const *argv[])
{
    cout << helper::add(1, 1) << endl;
    return 0;
}
```

```cpp title="CMakeLists.txt"
add_executable(main src/main.cpp)
target_link_libraries(main helper)
```

---

# test
- Test file example
- cmake file


```cpp title="test_helper"
#include <gtest/gtest.h>
#include <helper.h>

TEST(helper, add){
    EXPECT_EQ(0, helper::add(1, -1));
}
```

```cpp title="tests/CMakeList.txt"
add_subdirectory(/usr/src/gtest
    ${PROJECT_BINARY_DIR}/gtest
)

include(CTest)

set(TEST_BINARY ${PROJECT_NAME}_test)
add_executable(${TEST_BINARY} test_helper.cpp)
target_link_libraries(${TEST_BINARY}
    helper
    gtest
    gtest_main
)

add_test(
    NAME ${TEST_BINARY}
    COMMAND ${EXECUTABLE_OUTPUT_PATH}/${TEST_BINARY}
)
```

---

## root
- Root cmake file

!!! note ""
     root cmake must contain `enable_testing` cmake function
     for testing

```cpp title="tests/CMakeList.txt"
cmake_minimum_required(VERSION 3.10)
project(demo)

enable_testing()

add_subdirectory(helper)
add_subdirectory(app)
add_subdirectory(tests)
```

---

## usage
### command line

```bash
ctest -VV
#
...
test 1
    Start 1: demo_test

1: Test command: /home/user/projects/gtest_tutorial/build/tests/demo_test
1: Test timeout computed to be: 1500
1: Running main() from /usr/src/gtest/src/gtest_main.cc
1: [==========] Running 1 test from 1 test suite.
1: [----------] Global test environment set-up.
1: [----------] 1 test from helper
1: [ RUN      ] helper.add
1: [       OK ] helper.add (0 ms)
1: [----------] 1 test from helper (0 ms total)
1: 
1: [----------] Global test environment tear-down
1: [==========] 1 test from 1 test suite ran. (0 ms total)
1: [  PASSED  ] 1 test.
1/1 Test #1: demo_test ........................   Passed    0.00 sec
```

---

## VSCode
[VSCode marketplace](https://marketplace.visualstudio.com/items?itemName=matepek.vscode-catch2-test-adapter)

![](../images/cmake_test_icon.png)

## Usage

![](../images/vscode_test.png)
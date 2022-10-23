---
title: fmt
tags:   
    - cpp
    - format
    - fmt
---

## install
```
sudo apt install libfmt-dev
```

## Demo
```cpp title="fmt_hello.cpp"
#include <iostream>
#include <fmt/core.h>

using namespace std;

int main(int argc, char const *argv[])
{
    string s = "world";
    fmt::print("hello {}\n", s);
    auto data = fmt::format("{1} {0}", "hello", 1, 2, 3);
    cout << data << endl;
    return 0;
}
```

```c title="CMakeLists.txt"
find_package(fmt REQUIRED)

add_executable(fmt_hello fmt_hello.cpp)
target_link_libraries(fmt_hello PRIVATE fmt::fmt)
```

---

## Resource
- [{fmt} Formatting & Printing Library](https://hackingcpp.com/cpp/libs/fmt.html)


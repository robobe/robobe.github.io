---
title: spdlog hello
tags:
    - spdlog
---

## install
```
sudo apt install libspdlog-dev
```

## basic usage
```cpp title="hello_spd"
#include <spdlog/spdlog.h>

int main(int argc, char const *argv[])
{
    spdlog::set_level(spdlog::level::debug);
    spdlog::debug("debug {}", "more data");
    spdlog::info("info {} {}", "data", 1);
    spdlog::warn("warn");
    spdlog::error("error");
    
    return 0;
}
```

```c title="CMakeLists.txt"
find_package(spdlog REQUIRED)

add_executable(hello_spd hello_spd.cpp)
target_link_libraries(hello_spd PRIVATE spdlog::spdlog_header_only)
```

![](images/basic.png)
---
title: filesystem hello
tags:
    - cpp
---

- create_directories
- directory_iterator
- path
- filename
- extension
- stem
- exists

```cpp
#include <filesystem>
#include <iostream>
using namespace std;
namespace fs=std::filesystem;

int main(){
    std::string ppath = "/tmp/fs_demo";
    fs::create_directories(ppath);
    for (auto p: fs::directory_iterator(ppath))
    {
        std::cout << p.path() << std::endl;
        cout << fs::path(p.path()).filename() << endl;
        cout << fs::path(p.path()).extension() << endl;
        cout << fs::path(p.path()).stem() << endl;
    }

    cout << boolalpha << fs::exists(ppath) << endl;
    return 0;
}
```
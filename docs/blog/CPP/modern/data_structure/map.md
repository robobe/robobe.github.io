---
tags:
    - cpp
    - data structure
    - map
    - container
---

`std::map` is one of the most known data structures in C++, the default associative container
Associative containers are used when you have pairs of key/value and you want to find a value given its key.

```cpp title="map tutorial"
// Online C++ compiler to run C++ program online
#include <iostream>
#include <map>

int main() {
    // Write C++ code here
    
    std::map<int, std::string> data;
    // insert
    data[1] = "a";
    data[2] = "b";
    data.insert({3, "c"});
    
    // access
    std::cout << data[2] << std::endl;
    
    // erese
    data.erase(2);
    
    //find and check
    if (data.count(2) == 0){
        std::cout << "element not exists" << std::endl;
    }
    
    if (data.find(2) != data.end()){
        std::cout << "element found" << std::endl;
    }
    
    //size
    std::cout << data.size() << std::endl;
    
    //iter
    for(const auto &item: data){
        std::cout << item.first << ":" << item.second << std::endl;
    }
    
    
    return 0;
}
```
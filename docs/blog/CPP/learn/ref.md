---
title: C++ ref learn by doing
tags:
    - cpp
    - c++
---

- Ref are just reference to variable
- unlike pointer reference can't contain NULL value, they must reference to some variable
- They can't reassign, they don't have memory allocate for them


```cpp
#include <iostream>

#define LOG(x) std::cout << x << std::endl;

int main() {
    int a = 5;
    int& ref = a;
    ref = 2;
    LOG(a);
}
```

pass veritable as a reference to function

```cpp
#include <iostream>

#define LOG(x) std::cout << x << std::endl;

void counter(int& counter)
{
    counter++;
}

int main() {
    int a = 5;
    counter(a);
    LOG(a);
}
```
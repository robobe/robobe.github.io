---
title: Move 
tags:
    - cpp
---

Move object ownership

```cpp
#include <iostream>
#include <vector>

using namespace std;

int main(int argc, char const *argv[])
{
    string str = "Hello";
    vector<string> v;

    v.push_back(str);
    cout << "str After copy is: " << str << endl;

    // move str into vector , str might no be empty
    v.push_back(move(str));
    cout << "str After copy is: " << str << endl;
    return 0;
}

```
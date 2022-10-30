---
title: Smart pointer
tags:
    - cpp   
---

## Unique Pointers

- release it's memory when out of scope
- Unique pointer has no copy constructor
- Cannot be copied, **can be moved**
- Guarantees that memory is always owned by a single unique pointer

```cpp title="scope"
#include <iostream>
#include <memory>

using std::cout;
using std::endl;

class Basic{
public:
    Basic(){
        cout << "construct" << endl;
    }
    ~Basic(){
        cout << "de construct" << endl;
    }
    int a = 10;
};

void run(){
    auto a_ptr = std::make_unique<Basic>();
    cout << a_ptr->a << endl;
}
int main(int argc, char const *argv[])
{
    cout << "run call" << endl;
    run();
    cout << "run exit" << endl;
    return 0;
}
```

```cpp title="owner"
#include <iostream>
#include <memory>

using std::cout;
using std::endl;

class Basic{
public:
    int a = 10;
};

int main(int argc, char const *argv[])
{
    auto a_ptr = std::make_unique<Basic>();
    
    cout << a_ptr->a << endl;
    auto b_ptr = std::move(a_ptr);
    cout << b_ptr->a << endl;
    return 0;
}
```

---

## Smart pointer
- Store a usage counter and a raw pointer
  - Increases usage counter when copied
  - Decreases when destructed

```cpp
#include <iostream>
#include <memory>

using std::cout;
using std::endl;

class Basic{
public:
    Basic(int a): a_(a){
        cout << "construct" << endl;
    }
    ~Basic(){cout << "de construct" << endl;}
private:
    int a_;
};

int main(int argc, char const *argv[])
{
    auto a_ptr = std::make_shared<Basic>(10);
    cout << a_ptr.use_count() << endl;
    {
        // auto b_ptr = std::move(a_ptr);
        auto b_ptr = a_ptr;
        cout << b_ptr.use_count() << endl;
    }
    
    return 0;
}
```
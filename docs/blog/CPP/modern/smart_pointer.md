---
tags:
    - cpp
    - smart pointers

---
# Smart pointers
- unique
- shared
- weak
  

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

## shared pointer
- Store a usage `counter` and a raw pointer
  - Increases usage counter when copied
  - Decreases when copied exit scope
- release it's memory when `counter` zero

```cpp
#include <iostream>
#include <string>
#include <memory>

class Employee 
{
    std::string name;

public:
    Employee(std::string name): name(name)
    {
        std::cout << "constructor" << std::endl;
    }

    ~Employee()
    {
        std::cout << "destructure" << std::endl;
    }

    void print_me()
    {
        std::cout << "e->" << name << std::endl;
    }
};

int main()
{
    {
        auto ptr1 = std::make_shared<Employee>("Employee");
        std::cout << ptr1.use_count() << std::endl;
        {
            auto ptr2 = ptr1;
            std::cout << ptr1.use_count() << std::endl;
        }
        std::cout << ptr1.use_count() << std::endl;
        ptr1->print_me();
    }

    std::cout << "end" << std::endl;
    return 0;
}
```

---

## Weak pointers
- weak pointer don't keep it memory alive
- weak point are relevant (not expired) only if strong pointer still alive


```cpp
#include <iostream>
#include <string>
#include <memory>

class Employee 
{
    std::string name;

public:
    Employee(std::string name): name(name)
    {
        std::cout << "constructor" << std::endl;
    }

    ~Employee()
    {
        std::cout << "destructure" << std::endl;
    }

    void print_me()
    {
        std::cout << "e->" << name << std::endl;
    }
};

int main()
{
    {
        
        std::weak_ptr<Employee>  weak_ptr;
        {
            auto ptr1 = std::make_shared<Employee>("Employee");
            weak_ptr = ptr1;
            std::cout << ptr1.use_count() << std::endl;
        }
        std::cout << weak_ptr.expired() << std::endl;
    }

    std::cout << "end" << std::endl;
    return 0;
}
```

---

## Reference
- [SMART POINTERS in C++ (for beginners in 20 minutes)](https://youtu.be/e2LMAgoqY_k)
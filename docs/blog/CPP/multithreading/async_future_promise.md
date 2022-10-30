---
title: Async Future and Promise
tags:
    - multithreading
    - cpp
---

```cpp title="async_future.cpp"
#include <iostream>
#include <future>

using std::cout;
using std::endl;

int factorial(int N){
    int res = 1;
    for (int i = N; i > 1; i--)
    {
        res *= i;
    }
    return res;
    
}

int main(int argc, char const *argv[])
{
    std::future<int> fu = std::async(factorial, 4);
    //call with a profile , force a new thread
    //std::future<int> fu = std::async(std::launch::async, factorial, 4);

    // wait for result from async
    int x = fu.get();
    cout << x << endl;
    return 0;
}
```

### Async profile
- launch::deferred: Deferred the call in the same thread only when `get` call
- launch::async: Call the function in new thread
- launch::deferred | launch::async (default)

---

## promise

```cpp title="async_future_promise.cpp"
#include <iostream>
#include <future>
#include <chrono>
#include <thread>

using std::cout;
using std::endl;

int factorial(std::future<int>& f){
    int res = 1;
    // wait for promise value
    int N = f.get();
    for (int i = N; i > 1; i--)
    {
        res *= i;
    }
    return res;
}

int main(int argc, char const *argv[])
{
    std::promise<int> p;
    std::future<int> f = p.get_future();
    // promise a value in the future
    std::future<int> fu = std::async(std::launch::async, factorial, std::ref(f));
    
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    // send the promise value
    p.set_value(4);
    
    int x = fu.get();
    cout << x << endl;
    return 0;
}
```


---

# Reference
- [C++ Threading #7: Future, Promise and async()](https://youtu.be/SZQ6-pf-5Us)
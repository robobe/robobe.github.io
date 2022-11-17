---
title: Iterator and Generator hello
tags:
    - python
    - iterator
    - generator
---

Iterator in Python is simply an object that can be iterated upon. An object which will return data, one element at a time.

Python iterator object must implement two special methods (iterator protocol)
- __iter__()
- __next__()

!!! note "iterable"
    An object is called iterable if we can get an iterator from it.
    Container like `list` and `tuple` are iterable object

---

## Custom iterators

```python
class Base2():
    def __init__(self, max) -> None:
        self.__max = max
        self.__current = 0

    def __iter__(self):
        return self

    def __next__(self):
        if self.__current > self.__max:
            raise StopIteration()
        result = 2**self.__current
        self.__current += 1
        return result
```
     
!!! note "iter and next"
    `iter(obj)` and `next(obj)` method is the same like calling
    `obj.__next__()` and `obj.__iter__()` method
     

### iterator usage
- Iterator raise `StopIteration` exception when it's ended
  
```python
worker = Base2(2)
worker_iter = worker.__iter__()
print(next(worker_iter))
print(worker_iter.__next__())
print(next(worker_iter))
print(next(worker_iter))

1
2
4
Traceback (most recent call last):
  File "/home/user/projects/blog/examples/python/python/custom_iterators.py", line 22, in <module>
    print(next(worker_iter))
  File "/home/user/projects/blog/examples/python/python/custom_iterators.py", line 12, in __next__
    raise StopIteration()
StopIteration
```

---

# Generator

```python
def base2(max):
    for x in range(max):
        yield 2**x

print(base2(3))
for i in base2(3):
    print(i)

<generator object base2 at 0x7f8aa5cc07b0>
1
2
4
```

# Generator Expression

```python
g = (2**x for x in range(3))
print(g)
for i in g:
    print(i)

#
<generator object <genexpr> at 0x7f9af03247b0>
1
2
4
```

---

# Reference
- [How to make an iterator in Python](https://treyhunner.com/2018/06/how-to-make-an-iterator-in-python/)
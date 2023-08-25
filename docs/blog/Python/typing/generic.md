---
tags:
    - python
    - typing
    - generic
---

# Using Generics in python
Generic is a way to specify the type of data that variable can hold at use time and not when this type declare

```python
from typing import TypeVar, Generic, List, Any

T = TypeVar("T") #(1)
def first(items: List[T]) -> T: #(2)
    return items[0]

first_item: int = first([1, 2, 4])
```

1. The `TypeVar` declare that `T` can any type
2. The function `first` get list of type `T` and return `T`

---

## Generic class

```python
from typing import List, Generic, TypeVar

T = TypeVar("T")

class Items(Generic[T]):
    def __init__(self) -> None:
        self.__data: List[T] = []

    def add_item(self, item: T) -> None:
        self.__data.append(item)

    def first(self) -> T | None:
        if self.__data:
            return self.__data[0]
        return None


if __name__ == "__main__":
    items = Items[str]()
    items.add_item("a")

    other_items = Items[int]()
    other_items.add_item(1)
   
```


### Demo: Generic class with multiple generic types

```python
from typing import Generic, TypeVar

T = TypeVar("T")
U = TypeVar("U")


class MyClass(Generic[T, U]):
    def __init__(self, foo: T, bar: U) -> None:
        self.foo = foo
        self.bar = bar

    def get_foo(self) -> T:
        return self.foo

    def get_bar(self) -> U:
        return self.bar

if __name__ == "__main__":
    obj = MyClass[str, int]("name", 10)
    print(obj.get_foo())
```

---

# Reference
- [Using Generics in Python](https://medium.com/@steveYeah/using-generics-in-python-99010e5056eb)
- [Python’s covariance and contravariance](https://blog.magrathealabs.com/pythons-covariance-and-contravariance-b422c63f57ac)
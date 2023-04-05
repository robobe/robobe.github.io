---
tags:
    - python
    - typing
    - annotation
---

# Using Generics in python


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
    # items.add_item(1)

    other_items = Items[int]()
    other_items.add_item(1)
   
```

---

# Reference
- [Using Generics in Python](https://medium.com/@steveYeah/using-generics-in-python-99010e5056eb)
- [Python’s covariance and contravariance](https://blog.magrathealabs.com/pythons-covariance-and-contravariance-b422c63f57ac)
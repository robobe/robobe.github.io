---
tags:
    - typing
    - typevar
    - python
---

Type variables allow you to link several types together.

```python
from typing import TypeVar

T = TypeVar("T")

def foo(arg: T) -> T:
    return arg
```


# TypeVar

```python
from typing import TypeVar
T = TypeVar("T")

def foo(arg: T) -> T:
    return arg
```

```python
import decimal
from typing import TypeVar

Number = TypeVar("Number", int, float, decimal.Decimal)

def double(value: Number) -> Number:
    return value * 2

result: int
result = double(5)
```

---

## Reference
- [Advanced type annotations using Python's TypeVar](https://piccolo-orm.com/blog/advanced-type-annotations-using-python-s-type-var/)
- [`TypeVar`s explained ](https://dev.to/decorator_factory/typevars-explained-hmo)


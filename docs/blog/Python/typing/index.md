---
tags:
    - python
    - typing
    - annotation
---

# type hinting

```
from typing import (
    Any,
    Callable,
    Generic,
    Optional,
    Sequence,
    Type,
    TypeVar,
    Union,
    cast,
)
```

## List, Tuple, Dict

```python
from typing import List, Tuple, Dict

l: List[str] = ["a", "b", "c"]
t: Tuple[int, str] = (1, "a")
d: Dict[str, int] = {"a": 1}
```

## overload

```python
from collections.abc import Sequence
from typing import overload


@overload
def double(input_: int) -> int:
    ...


@overload
def double(input_: Sequence[int]) -> list[int]:
    ...


def double(input_: int | Sequence[int]) -> int | list[int]:
    if isinstance(input_, Sequence):
        return [i * 2 for i in input_]
    return input_ * 2

if __name__ == "__main__":
    print(double(1))
    print(double([1, 2, 3]))
```

## generic


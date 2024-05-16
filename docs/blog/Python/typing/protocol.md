---
tags:
    - python
    - typing
    - protocol
    - generic
---
The Python type system supports two ways of deciding whether two objects are compatible as types: 
- nominal subtyping 
- structural subtyping.

**Nominal subtyping** is strictly based on the class hierarchy
**Structural subtyping** is based on the operations that can be performed with an object. Structural subtyping can be seen as a static equivalent of duck typing,


## Demo

- FastRate not implement the `IRate` protocol

```python title="protocol_demo.py"
from typing import Protocol

class IRate(Protocol):
    def update(self):
        ...


class SlowRate():
    def update(self):
        ...

class FastRate():
    def no_update(self):
        ...


def calc_rate(rate: IRate):
    rate.update()

calc_rate(SlowRate())
calc_rate(FastRate())
```

```bash title="mypy checking"
mypy protocol_demo.py

#
error: Argument 1 to "calc_rate" has incompatible type "FastRate"; expected "IRate"
```

### Another example

```python title="another_protocol_demo.py"
from typing import Protocol


class IRate(Protocol):
    def update(self): ...


class SlowRate:
    def update(self): ...


class FastRate:
    def no_update(self): ...


slow_rate: IRate = SlowRate()
fast_rate: IRate = FastRate()
```

```bash title="mypy checking"
mypy protocol_demo.py

#
 error: Incompatible types in assignment (expression has type "FastRate", variable has type "IRate")
```

---

### Demo: Run time check
Using runtime_checkable decorator we can check if type implement the protocol


```python title="runtime_protocol_demo.py"
from typing import Protocol, runtime_checkable

@runtime_checkable
class IRate(Protocol):
    def update(self): ...


class SlowRate:
    def update(self): ...


class FastRate:
    def no_update(self): ...


assert isinstance(SlowRate(), IRate)
assert isinstance(FastRate(), IRate)

```


---


!!! tip "VSCode mypy config"
    ```json title="settings.json"
    "python.linting.mypyEnabled": true
    ```
     
!!! tip "VSCode mypy config"
    The above setting mark as deprecated
    For know it work better then the extension alternative


!!! tip "more mypy config from vscode settings"
    ```json
        {
        "python.linting.mypyEnabled": true,
        "python.linting.mypyArgs": [
            "--ignore-missing-imports",
            "--follow-imports=silent",
            "--show-column-numbers",
            "--allow-untyped-defs",
            "--allow-subclassing-any",
            "--allow-untyped-calls",
            "--strict"
            ]
    }
    ```
---

## Demo: Protocol with generic

```python
from typing import TypeVar
from typing_extensions import Protocol

T = TypeVar("T")


class SupportAdd(Protocol[T]):
    def add(self, a: T, B: T) -> T:
        pass


class SimpleAdd:
    def add(self, a: int, b: int) -> int:
        return a + b


class SimpleAdd2:
    def add(self, a: str, b: str) -> str:
        return a + b


simple_add: SupportAdd[int] = SimpleAdd()
simple_add2: SupportAdd[str] = SimpleAdd2()
```

---

## Reference
- [mypy Protocols and structural subtyping](https://mypy.readthedocs.io/en/stable/protocols.html)
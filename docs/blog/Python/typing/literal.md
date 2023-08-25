---
tags:
    - python
    - typing
    - literal
    - mypy
---

The Literal type was introduced in [PEP 586](https://peps.python.org/pep-0586/) as a way to indicate that a variable must have a set of certain values.


## Demo

```python
from typing import Literal


Status = Literal[404, 200, 500]

def set_x(value: Status):
    print(value)

set_x(404)
set_x(1404)
```

#### mypy error
```bash
Argument 1 to "set_x" has incompatible type "Literal[1404]"; expected "Literal[404, 200, 500]"mypy(error)
```
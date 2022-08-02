---
title: PyTest - Mocking
tags:
    - pytest
    - mock
---

!!! note "mocking"
    A mock object is a simulated object that mimics the behavior of the smallest testable parts of an application in controlled ways. It's replace of one or more function or objects calls

    A mock function call return a predefined value immediately without doing any work

In Python mocking implement by `unittest.mock` module

### Simple demo

```bash title="project"
search
  ├── tutorial
  │ ├── __init__.py
  │ └── demo.py
  └── tests
   └── test_demo.py
```

```python title="demo.py"
# method to mock
def get_number() -> int:
    return 5

# function under test
def add(a: int) -> int:
    b = get_number()
    return a + b
```
     
```python title="test_demo.py"
from unittest.mock import patch, MagicMock

@patch("tutorial.demo.get_number")
def test_add_mock(mock_get_number: MagicMock) -> None:
    mock_get_number.return_value = 2
    result = add(1)
    assert result == 3
```

!!! warning
    `@path` full name of the function or class to patch `module_name.func_name` for example to path `get_number` function in `demo` module. `@patch("demo.get_number")`
     
---

### More complex demo
```python title="demo.py"
class Generator():
    def build(self) -> int:
        return 5

def get_number() -> Generator:
    gen = Generator()
    return gen

# function under test
def add(a: int) -> int:
    b = get_number().build()
    return a + b
```

- mock get_number function
- mock Generator class

```python title="test_demo.py"
from py_template import demo

from unittest.mock import patch, MagicMock

@patch("tutorial.demo.get_number")
def test_add_mock(mock_get_number: MagicMock) -> None:
    # mock Generator obj by create MagicMock
    # mock build method 
    mock_generator = MagicMock()
    mock_generator.build.return_value = 2
    # mock get_number function
    mock_get_number.return_value = mock_generator
    result = demo.add(1)
    assert result == 3
```

---

## MagicMock
Provide a simple mocking interface that allow to mock partial real object that we wont to patch

`return_value`  allows you to choose what the patched **callable** returns,
usually  we return the same type of the real callable but controllable


---

# Reference
- [Mock's return_value & side effect](https://thedmitry.pw/blog/2020/12/mocks-side-effect/)
- [ Unit Testing in Python with pytest | Introduction to mock (Part-9) ](https://youtu.be/dw2eNCzwBkk)
- [Unit Testing in Python with pytest | Advanced Mocking (Part-10)](https://youtu.be/M46H4GIdfl0)
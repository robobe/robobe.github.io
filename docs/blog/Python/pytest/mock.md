---

tags:
    - pytest
    - mock
    - unittest
---
# PyTest - Mocking

!!! note "mocking"
    A mock object is a simulated object that mimics the behavior of the smallest testable parts of an application in controlled ways. It's replace of one or more function or objects calls

    A mock function call return a predefined value immediately without doing any work

In Python mocking implement by `unittest.mock` module


## demo1
[How To Mock Patch A Function](https://youtu.be/ClAdw7ZJf5E?list=PLJsmaNFr5mNqSeuNepT3IaMrgzRMm9lQR)

```python title="demo.py"
from random import randint

def func_under_test():
    r = randint(1, 8)
    return r
```

```python title="test_demo.py"
from demo import func_under_test
from unittest import mock

@mock.patch("demo.randint", return_value=7, autospec=True)
def test_with_mock(mock_randint):
    r = func_under_test()
    assert r == 7
```

!!! tip "patch"
    Patch `the thing` where it used 
    not where it's import
    In the above example we patch `demo.randint` and **not** `random.randint`
     

!!! tip "autospec"
    [autospec](https://stackoverflow.com/questions/35915703/when-using-unittest-mock-patch-why-is-autospec-not-true-by-default)


---

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

## Mock with parametrize
- Set mock **return_value** from parameter


```python title="demo.py"
from random import randint

def get_random():
    return randint(1, 10)


def good_random():
    r = get_random()
    if r > 7:
        return "win"
    else:
        return "lose"
```

```python title="test_demo.py"
import pytest
from unittest import mock
from demo import good_random


@pytest.mark.parametrize("_input, expected", 
    [
        (8, "win"),
        (5, "lose")
    ])
@mock.patch("demo.get_random")
def test_good_random(mock_get_random, _input, expected):
    mock_get_random.return_value = _input
    result = good_random()
    assert result == expected

```

---

## MagicMock
Provide a simple mocking interface that allow to mock partial real object that we wont to patch

#### return_value
allows you to choose what the patched **callable** returns,
usually  we return the same type of the real callable but controllable

#### side_effect
Change the behavior of the mock

##### side_effect = Iterable
yield the values from defined iterable on subsequent call

```bash
>>> from unittest.mock import MagicMock
>>> m = MagicMock()
>>> m.get_data.side_effect = [5, 10, 15]
>>> m.get_data()
5
>>> m.get_data()
10
>>> m.get_data()
15
```

```python
from unittest.mock import patch

def my_input() -> int:
    return 1

def method_to_test():
    a = my_input()
    b = my_input()
    return a+b


@patch("test_demo.my_input")
def test_multiple(mock_my_input):
    mock_my_input.side_effect = [1, 2]
    result = method_to_test()
    assert result == 3
```

##### side_effect = Exception

```bash
m.check.side_effect = Exception("custom exception")
>>> m.check()
Traceback (most recent call last):
  File "<stdin>", line 1, in <module>
  File "/usr/lib/python3.8/unittest/mock.py", 
  ...
    raise effect
Exception: custom exception
```

##### ide_effect = callable

The callable will be executed on each call with the parameters passed when calling the mocked method

```bash
>>> def call_me(name):
...     print(name)
... 
>>> m.run_call.side_effect = call_me
>>> m.run_call("a")
a
>>> m.run_call("b")
b
>>> m.run_call.call_count
2
>>> m.run_call("b")
b
>>> m.run_call.call_count
3
```


---

# Reference
- [Mock's return_value & side effect](https://thedmitry.pw/blog/2020/12/mocks-side-effect/)
- [ Unit Testing in Python with pytest | Introduction to mock (Part-9) ](https://youtu.be/dw2eNCzwBkk)
- [Unit Testing in Python with pytest | Advanced Mocking (Part-10)](https://youtu.be/M46H4GIdfl0)
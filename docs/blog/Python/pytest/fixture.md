---
tags:
    - pytest
    - unittest
    - python
---
# Pytest fixture

Fixture are function that help us to **arrange** the domain/ environment before test
and then help us to **clean** the mess after the test


## Demo
- Use fixture for multiple tests


```python title="demo.py"
def hello(name):
    return f"hello {name}"

def bye(name):
    return f"bye {name}"
```

```python title="test_demo.py"
# fixture demo
from demo import hello, bye
import pytest

@pytest.fixture
def name():
    return "me"

def test_hello(name):
    result = hello(name)
    assert result == "hello me"

def test_bye(name):
    result = bye(name)
    assert result == "bye me"
```

!!! note "fixture"
    pytest look for matching fixture name 
     
---

## Demo II
- Use fixture as setup and teardown/clean


```python
from typing import Callable
from typing import Iterator
import pytest

@pytest.fixture
def setup() -> None:
    print("run setup fixture before test")

@pytest.fixture
def setup_teardown() -> Iterator[None]:
    print("run setup fixture before test")
    yield
    print("run teardown fixture after test")


def test_with_fixture(setup: Callable) -> None:
    print("run test")
    assert True

def test_with_fixture_II(setup_teardown: Callable) -> None:
    print("run testII")
    assert True
```

!!! Tip "run pytest verbose"
    ```
    python -m pytest -s <test file>
    ```


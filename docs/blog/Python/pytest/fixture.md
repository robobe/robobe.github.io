---
title: Pytest fixture
tags:
    - pytest
    - unittest
    - python
---

Fixture are function that help us to **arrange** the domain/ environment before test
and then help us to **clean** the mess after the test

These are accessed by test functions through arguments see demo


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


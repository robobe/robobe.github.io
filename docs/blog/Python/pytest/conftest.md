---
title: Pytest conftest
tags:
    - pytest
    - unittest
    - python
---

Using `conftest.py` to share fixtures and hooks among all tests

---

## autouse

```python title="conftest.py"
from typing import Iterator
import pytest


@pytest.fixture(autouse=True)
def setup_teardownII() -> Iterator[None]:
    print("run setup fixture before test")
    yield
    print("run tear down fixture after test")
```

!!! note "autouse"
    Set `autouse` to run this fixture for all tests
     


```python title="test_file.py"
def test_with_fixture() -> None:
    print("run test III")
    assert True
```

!!! Tip "run pytest verbose"
    ```
    python -m pytest -s <test file>
    ```
---

### Run tests
```bash
python -m pytest -s test_file.py
#
run setup fixture before test
run test III
.run tear down fixture after test
```

---

## scope

[Fixture scopes](https://docs.pytest.org/en/6.2.x/fixture.html)

Fixtures are created when first requested by a test, and are destroyed based on their scope:

- function: (default) The fixture is destroy at the end of the test,
- class: 
- module:
- package:
- session: The fixture destroy at the end of the last session.


### scope demo

```python title="conftest.py"
from typing import Iterator
import pytest


@pytest.fixture(scope="session", autouse=True)
def setup_teardownII() -> Iterator[None]:
    print("\nrun setup fixture before session start")
    yield
    print("run tear down fixture after session end")
```

```python title="test_file.py"
def test_with_fixture() -> None:
    print("run test")
    assert True

def test_with_fixture_II() -> None:
    print("run testII")
    assert True

def test_with_fixtureIII() -> None:
    print("run test III")
    assert True
```

#### run

```bash linenums="1" hl_lines="3 7"
python -m pytest -s tests/test_file.py
#
run setup fixture before session start
run test
.run testII
.run test III
.run tear down fixture after session end
```
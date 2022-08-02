---
title: pytest parameterize tests
tags:
    - pytest
    - unittest
    - python
---

## fixture

```python title="test_with_params.py"
from typing import Callable
import pytest
from _pytest.fixtures import SubRequest

@pytest.fixture(params=[10, 20, 30])
def demo_fixture(request: SubRequest) -> None:
    print(f"\nrun test with param: {request.param}")


def test_with_fixture(demo_fixture: Callable) -> None:
    print("run test")
    assert True
```


```bash
python -m pytest -s tests/test_with_params.py
#
tests/test_with_params.py 
run test with param: 10
run test
.
run test with param: 20
run test
.
run test with param: 30
run test
.
```

---

## marker.parametrize

```python title=""
from typing import Callable
import pytest
from _pytest.fixtures import SubRequest


@pytest.mark.parametrize("a, b, result", [(1, 2, 3), (5, 5, 11), (2, 2, 4)])
def test_add(a: int, b: int, result: int) -> None:
    assert a + b == result

```

#### run test

```bash
python -m pytest -s -v tests/test_with_params.py
#
tests/test_with_params.py::test_add[1-2-3] PASSED
tests/test_with_params.py::test_add[5-5-11] FAILED
tests/test_with_params.py::test_add[2-2-4] PASSED

====================================================================== FAILURES =======================================================================
__________________________________________________________________ test_add[5-5-11] ___________________________________________________________________

a = 5, b = 5, result = 11

    @pytest.mark.parametrize("a, b, result", [(1, 2, 3), (5, 5, 11), (2, 2, 4)])
    def test_add(a: int, b: int, result: int) -> None:
>       assert a + b == result
E       assert 10 == 11
E         -10
E         +11

tests/test_with_params.py:18: AssertionError
========================================================= 1 failed, 2 passed in 0.03 seconds
```
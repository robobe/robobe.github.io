---
tags:
    - pytest
    - parameterize
    - python
---
# pytest parameterize tests

## marker.parametrize

```python
import pytest


@pytest.mark.parametrize("a, b, result", 
    [
        (1, 2, 3),
        (5, 5, 10),
        (2, 2, 4)
    ])
def test_add(a: int, b: int, result: int) -> None:
    assert a + b == result

```

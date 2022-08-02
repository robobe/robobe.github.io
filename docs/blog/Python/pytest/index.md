---
title: Pytest 
tags:
    - pytest
    - unittest
    - python
---

# Test
- Arrange (setup)
- Act (object/method under test)
- Assert
- Cleanup


# Groping Tests
## Custom markers
- Using `pytest.mark` decorator we can group tests 

Idea for custom marker  
- sanity
- regression


```python
import pytest

@pytest.mark.g1
def test_demo():
    assert True
```

```
python -m pytest -m g1 g2
```

---
## Builtin markers

```bash
pytest --markers

...
@pytest.mark.skip(reason=None)
@pytest.mark.skipif(condition, ..., *, reason=...)
@pytest.mark.xfail(condition, ..., *, reason=..., run=True, raises=None, strict=xfail_strict)
...
```

### xfail
mark the test function as an expected failure

for example if function under test not implement yet
---

# Reference
- [PyTest for Absolute Beginners](https://www.youtube.com/playlist?list=PLL34mf651faNqwhZEM91zU3c-dcc4dLF0)
- [pytest IndianPythonista](https://www.youtube.com/c/IndianPythonista)
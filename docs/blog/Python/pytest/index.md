---
title: Pytest 
tags:
    - pytest
    - unittest
    - python
---

# Test

## Tests structure
- Arrange (setup)
- Act (object/method under test)
- Assert
- Cleanup


## simple demo

!!! note
    pytest search all module that naming start with `test` prefix and method start with `test` prefix
    
     
```python title="test_demo.py"
def test_func():
    assert 1 == 1

def test_func_fail():
    assert 1 == 2
```

```bash
# from project root folder run
# run all tests
pytest

# run module tests
pytest test_demo.py

# run specific test from module
pytest test_demo.py::test_func
```

## 
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
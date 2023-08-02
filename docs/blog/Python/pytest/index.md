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

```bash title="run test and output to stdout"
pytest -s test_demo.py::test_func
```


## parts
- [fixture](fixture.md)
- [mock](mock.md)
- [params](params.md)
- [conftest](conftest.md)
- [logging](logging.md)
---

## Reference
- [Unit Testing in Python with pytest](https://www.youtube.com/playlist?list=PLyb_C2HpOQSBWGekd7PfhHnb9GnqDgrxS)
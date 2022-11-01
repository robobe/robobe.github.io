---
title: Add test to your package
tags:
    - pytest
    - test
---

## Minimal project structure
```
py_tutorial_pkg/
  py_tutorial_pkg/
      __init__.py
      demo.py
  package.xml
  setup.cfg
  setup.py
  tests/
      test_demo.py
```

```python title="setup.py"
# Add line to setup.py
tests_require=['pytest'],
```

```python title="py_tutorial_pkg/demo.py
def func_under_test(a, b):
    return a+b
```

```python title="tests/test_demo.py"
from py_tutorial_pkg import demo

def test_math():
    result = demo.func_under_test(2, 2)
    assert result == 4
```

---

## usage

```bash
# Run test in verbose mode

colcon test --packages-select py_tutorial_pkg \
--event-handler=console_direct+ 
```

- Run specific test
```bash
# colcon test --packages-select <name-of-pkg> --pytest-args -k name_of_the_test_function

colcon test --packages-select py_tutorial_pkg \
--pytest-args -k test_math \
--event-handler=console_direct+ 
 ```
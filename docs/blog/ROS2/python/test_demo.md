---
tags:
    - pytest
    - test
    - ros2
    - rclpy
---
# Add test to your package

unit test without ROS: unittest/pytest
unit test with ROS: launch_test + unittest
Integration Testing:  launch_test + unittest

!!! tip Run tests
    All test run with `colcon test`


     

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

!!! note "colcon event-handler"
    The event handler used to generate any kind of output base on the progress of the invocation  
      
    use `+` sign to enable handler and `-` to disabled

!!! note "event-handler"
    - **console_direct**: Pass output directly to `stdout`, `stderror` 
    - console_cohesion: Pass job output at once after it has finish

### check result
```
colcon test-result --all
```
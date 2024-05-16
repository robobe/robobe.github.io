---
tags:
    - pytest
    - test
    - ros2
    - rclpy
    - python
---
# ROS2 Test using pytest
- Unit Test
- Integration Test
- E2E Tests

## Unit Test
Using python pytest/ unittest framework
unit test without ROS: unittest/pytest
unit test with ROS: launch_test + unittest

## Integration Test
Test multiple node at once using by implement `launch_test + unittest` and run it using `colcon test`

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

```python title="py_tutorial_pkg/demo.py"
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
- Run package tests
- Check results
- Run selected test


```bash
# build package
colcon build --symlink-install --packages-select py_tutorial_pkg
# Test package
colcon test --packages-select  py_tutorial_pkg
```

### Check

```bash
# Check summary
colcon test-result --all

# Verbose result
colcon test-result --all --verbose
```


```bash
# colcon test --packages-select <name-of-pkg> --pytest-args -k name_of_the_test_function

colcon test --packages-select simples \
--pytest-args -k test_math \
--event-handler=console_direct+ 
```

!!! note "colcon event-handler"
    The event handler used to generate any kind of output base on the progress of the invocation  
      
    use `+` sign to enable handler and `-` to disabled

!!! note "event-handler"
    - **console_direct**: Pass output directly to `stdout`, `stderror` 
    - console_cohesion: Pass job output at once after it has finish
 test-result --all
```

---

```
```
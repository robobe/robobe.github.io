---
title: ament_mypy
tags:
    - linter
    - ros2
---

## install 

```bash
sudo apt install ros-humble-ament-cmake-mypy
```

## usage
!!! warning "cmake"
    if using cmake check `Linting ROS 2 Packages with mypy` in reference
     
- Add testcase to `test` folder
- config `setup.py`

---

### setup.py

Add / Edit line

```python
tests_require=['pytest']
```

### TestCase
```python title="test_mypy.py"
from ament_mypy.main import main

import pytest


@pytest.mark.mypy
@pytest.mark.linter
def test_mypy():
    rc = main()
    assert rc == 0, 'Found code style errors / warnings'
```

## Run
```bash
colcon test --packages-select <YOUR_PACKAGE> --event-handlers console_direct+
```
---

# Reference
- [Linting ROS 2 Packages with mypy](https://ubuntu.com/blog/linting-ros-2-packages-with-mypy)
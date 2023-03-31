---
tags:
    - linter
    - ros2
    - mypy
---
# Linting ROS 2 Packages with mypy
**linter**, is a static code analysis tool used to flag programming errors, bugs, stylistic errors and suspicious constructs.  


**mypy** is an optional static type checker for Python that aims to combine the benefits of dynamic (or "duck") typing and static typing

## python hinting 
Type hints are performed using `Python annotation` 
They are used at add types to variables function arguments and return values,  
Python hinting has no runtime effect they only use for static analyses with tools like `mypy` and help ide's to help us with more information and intellisense

[Post from Python tutorial: Introduction to Python type hints](https://www.pythontutorial.net/python-basics/python-type-hints/)

```python title="hinting demo"
from typing import List
from typing import Union

number = Union[int, float]

def get_list(item: number) -> List[number]:
    list = [1, 2, item]
    return list

print(get_list(1.0))

```

## VSCode 
Config mypy 

```json title="setting.json"
"python.linting.mypyEnabled": true,
"python.linting.mypyCategorySeverity.error":"Warning",
"python.linting.mypyArgs": [
    "--ignore-missing-imports"
]
```

---

## ROS2 linting
The `ament_lint` metapackage defines many common linters that can **integrate into the build/test pipeline for ROS 2**. The package ament_mypy within handles mypy integration
## install

```bash
sudo apt install ros-humble-ament-cmake-mypy
```

## mypy demo
- Config package.xml
- Edit setup.py
- Add TestCase

### package.xml
- Add entry
  
```xml
<test_depend>ament_mypy</test_depend>
```
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

### Run (colcon test)

```bash
colcon test --packages-select <YOUR_PACKAGE> --event-handlers console_direct+
```

---

## mypy ini
[mypy ini read the doc for more settings](https://mypy.readthedocs.io/en/stable/config_file.html)

!!! tip "ROS2 ament mypy"
    `ini` file location at `test` folder for `colcon test`
     

```ini title="mypy.ini"
[mypy]
ignore_missing_imports = True
```

#### ignore_missing_imports
Suppresses error messages about imports that cannot be resolved.

#### show_column_numbers
Shows column numbers in error messages.

---

# Reference
- [Linting ROS 2 Packages with mypy](https://ubuntu.com/blog/linting-ros-2-packages-with-mypy)
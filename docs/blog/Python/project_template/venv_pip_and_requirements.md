---
title: Python project template - venv
description: venv, pip and requirements handling
date: "2022-24-05"
banner: images/python.png
tags:
    - python
    - project
---

## venv
```bash title="create"
python -m venv venv
```

```bash title="usage"
# Clear all outside reference
unset PYTHONPATH
source venv/bin/activate
```

!!! tip "virtual env"
    Clear all other reference to python libraries by unset `PYTHONPATH` variable

## pip

!!! tip "auto complation"
    ```bash
    pip complation --bash >> ~/.bashrc
    source ~/.bashrc
    ```

## Dependencies
- Using pip to install project dependencies
- Separate dependencies to two or more files
  - dev packages
  - prod packages(reference prod file from setup.py)
  - project source reference

### project
- Install / Reference project from virtualenv

```
pip install -e .
```

!!! tip 
    `pip install -e . ` require setup.py file

---

### requirements

!!! tip
    Call `prod` file from `dev` using `-r` option
    ```
    -r requirements.txt
    ```
#### prod
```txt title="requirements.txt"
# Add project requirements here
```

#### dev
```txt title="requirements-dev.txt"  hl_lines="1"
-r requirements.txt
black
mypy
isort
```

```bash title="usage"
pip install -r requirements-dev.txt
```

---

### setup.py

```python title="minimal setup"
from setuptools import setup, find_packages

setup(
    name='MyPackageName',
    version='1.0.0',
    url='https://github.com/mypackage.git',
    author='Author Name',
    author_email='author@gmail.com',
    description='Description of my package',
    packages=find_packages(),    
    install_requires=[],
)
```



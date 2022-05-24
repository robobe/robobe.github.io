---
title: Python project template 
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
  - dev
  - prod (reference prod file from setup.py)

### requirements files
- , call prod file from dev using `-r` option

```txt title="requirements.txt"
# Add project requirements here
```

```txt title="requirements-dev.txt"  hl_lines="1"
-r requirements.txt
black
mypy
isort
```

```bash title="usage"
pip install -r requirements-dev.txt
```

### using setup.py



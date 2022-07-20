---
title: Python project template - unittest
description: tests using pytest
date: "2022-24-05"
banner: images/python.png
tags:
    - python
    - project
    - template
    - unittest
    - pytest
---

## Install and Setup
- Add `pytest` to `requirements-dev.txt`  
- Run `pip install -r requirements-dev.txt` again
- Create `tests` sub folder under root project folder
- Add tests modules files prefix with `test_`
- Add function to modules start also with `test_` prefix
- Install project using `pip install -e .`
- Run `pytest`

## demo
demp_proj project 

```
py_template/
├── .vscode
├── py_template
|    └── example.py
├── setup.cfg
├── setup.py
├── tests
|    └── test_example.py
├── venv
├── requirements.txt
├── requirements-dev.txt
└── version.py

```




```python title="test_example.py"
--8<-- "/home/user/projects/py_template/tests/test_example.py"
```

```python title="example.py"
--8<-- "/home/user/projects/py_template/py_template/example.py"
```

---


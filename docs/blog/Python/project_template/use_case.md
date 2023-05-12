---
tags:
    - python
    - project
    - template
    - from a-z
---

# Python project - demo
- Create project
- Init git
- Add virtual env. install project in `development mode`
- Install dependencies from requirements-dev.txt

## Steps
- Create root project folder
- Create virtual environment
  - Source it
- Create project src sub folder (same as project name)
- Add `__init__.py`
- At root project folder add
  - setup.py
  - .gitignore
  - .pylintrc
  - requirements.txt
  - requirements-dev.txt
  - README.md
- Init git
- Add and commit files
- Add tests folder
  - Write first unit test
  - Install `pytest` package
  - Config VSCode for using `pytest`
- Tools set
  - black
  - isort
  - flake8
  - pylint
  - mypy

```title="project struct"
├── py_commonlib
│   ├── hello.py
│   ├── __init__.py
├── requirements-dev.txt
├── requirements.txt
├── setup.py
├── README.md
└── tests
│    └── test_demo.py
├── .gitignore
├── .pylintrc
└── venv
```

```bash title="venv"
python -m venv venv
source venv/bin/activate
```

### requirements
```title="requirements"
```

```title="requirements-dev.txt"
-r requirements.txt
black
pytest
```

- Install dev and runtime packages
```
(venv)python -m pip install -r requirements-dev.txt
```

- Install project in `development mode`
```
(venv)python -m pip install -e .
```

!!! note development mode
    Create symlink between virtual env `site-packages` to project folder
    ```bash
    # /venv/lib/python3.10/site-packages 
    py-commonlib.egg-link
    ```

!!! note "to check"
    why `development mode` create `xxx.egg-info` folder at project root
     

### setup.py
- Set/Read `install_requires` from `requirements.txt`


!!! note "Dev packages"
    dev packages install from `requirements-dev.txt`
    At can be declare and install from setup.py.
    I preferred to config only runtime dependencies in `setup.py`
     
    ```python title="setup.py"
    extras_require={"dev": ["flake8", "mypy", "black", "isort"]}
    ```

    ```bash title="install"
    # bash
    pip install -e .[dev]
    # zsh
    pip install -e ".[dev]"
    ```

```python
from setuptools import setup, find_packages

with open("requirements.txt", "r", encoding="utf-8") as f:
    required = f.read().splitlines()

PACKAGE_NAME = "py_commonlib"
setup(
    name=PACKAGE_NAME,
    version="0.0.1",
    author="Author Name",
    author_email="author@gmail.com",
    description="python pkg for testing python project template",
    packages=find_packages(),
    install_requires=required,
)

```
     
---

### Tests
- each `test` file start with `test` prefix

#### run from cli
```bash title="run tests"
python -m pytest
```
---

## git submodules
[Using Git Submodule and Develop Mode to Manage Python Projects](https://www.codeproject.com/Articles/5254848/Using-Git-Submodule-and-Develop-Mode-to-Manage-Pyt)

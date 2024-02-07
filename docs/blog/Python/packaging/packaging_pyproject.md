---
tags:
    - python
    - packaging
    - pyproject
    - setup
---

## old setup.py

## new
Standardized way to specify package metadata in a pyproject.toml file and the way to build packages from source code using that metadata using **backend**

#### TOML
Support date, float and integer
TOML support comments using `#`, `"""\ """`


#### backend
The backend reads your pyproject.toml and actually does the work of turning your source code into a package archive that can be installed or distributed
for example:
- setuptoos (<61)
- poetry-core

#### frontend
The frontend is just a user interface  that calls the backend.
for example
- pip
- poetry
- pdm


## Demo
### Project
```
```

```toml title="pyproject.toml"
[project]
name = "test_py"
version = "0.1"

[build-system]
requires = ["setuptools"]
build-backend = "setuptools.build_meta"
```

```bash
# from venv
pip install -e .
```

### build
using [build](https://pypi.org/project/build/) package to create tar and whl files

```bash
pip install build
```

```bash
python3 -m build
```

check `dist` folder for `tar` and `whl` files

---

## Demo
Add metadata, dependencies, entry points and none python files

### metadata
[PEP 621 – Storing project metadata in pyproject.toml](https://peps.python.org/pep-0621/)

```toml
[project]
name = "py_test"
version = "0.1"
description = "This is a demo package"
readme = "README.md"
requires-python = ">=3.10"
```

### none python files and folder

```toml
[tool.setuptools.package-data]
# package_name = [folder, file ..]
test_py = ["static/*", "logging.json"]
```


### entry point

```toml
[project]
name = "test_py"
version = "0.1"
scripts = { demo_test_py = "test_py.app:main" }
```

build, install and run demo_test_py


### dependencies
```toml
[project]
# ...
dependencies = [
    "coloredlogs"
]
```

#### optional dependencies
```toml
[project.optional-dependencies]
test = ["pytest"]
```

```
pip install test_py[test]
pip install -e .[test]
```


---

## Reference
- [Python Packaging Best Practices](https://medium.com/@miqui.ferrer/python-packaging-best-practices-4d6da500da5f)
- [Python packages with pyproject.toml and nothing else](https://til.simonwillison.net/python/pyproject)
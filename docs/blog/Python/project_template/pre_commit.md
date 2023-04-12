---
tags:
    - python
    - project
    - template
    - pre-commit
    - git-hook
    - isort
    - black
    - mypy
    - flake8
---

# Python project template - pre-commit
Demo of usage pre-commit tool

Install pre-commit tool and config it with four tools

- isort
- black
- mypy
- flake8


## pre-commit

- Install pre-commit
- Add config file
- Install hooks
- Add hooks
- Run


```bash title="install"
python -m pip install pre-commit
```

```bash title="config"
touch .pre-commit-config.yaml
```

```bash title="install hook"
pre-commit install
```

### hooks
#### isort
isort is a Python utility / library to sort imports alphabetically, and automatically separated into sections and by type.

- config pre-commit
- add isort config to `pyproject.toml`
- install and run


```yaml title=".pre-commit-config.yaml"
repos:
  - repo: https://github.com/PyCQA/isort
    rev: 5.12.0
    hooks:
      - id: isort
```

```ini title="pyproject.toml"
[settings]
[tool.black]
line-length = 120

[tool.isort]
profile = "black"
```

```bash title="install and run"
pre-commit install
# Run
pre-commit run -a
```

!!! tip "config with black"
     config isort with `black` profile


---

#### black
Black is a PEP 8 compliant opinionated formatter. Black reformats entire files in place.

- config pre-commit
- add isort config to `pyproject.toml`
- install and run

```yaml title=".pre-commit-config.yaml"
repos:
  - repo: https://github.com/psf/black
    rev: 23.3.0
    hooks:
      - id: black
        language_version: python3.10
```

```ini title="pyproject.toml"
[settings]
[tool.black]
line-length = 120

[tool.isort]
profile = "black"
```

```bash title="install and run"
pre-commit install
# Run
pre-commit run -a
```

!!! tip "config isort"
     config isort with `black` profile

     black and isort format import different,
     if both config as pre-commit action they interrupt each other

!!! tip "disable fmt format for code section"
    ```
    # fmt: off
    ...
    # fmt: on
    ```

---

#### mypy

- config pre-commit
- add mypy section to `pyproject.toml` config
- install and run

```yaml title=".pre-commit-config.yaml"
repos:
  - repo: https://github.com/pre-commit/mirrors-mypy
    rev: v0.942
    hooks:
      - id: mypy
```

```ini title="pyproject.toml"
[settings]
[tool.black]
line-length = 120

[tool.mypy]

```

!!! note "add inline ignore"
    ```python
    # type: ignore
    ```
     

#### flake8
- config pre-commit
- add `.flake8` config


code style checker for PEP8

```yaml title=".pre-commit-config.yaml"
repos:
  - repo: https://github.com/PyCQA/flake8
    rev: 6.0.0
    hooks:
      - id: flake8
```

```ini title=".flake8"
[flake8]
max-line-length = 120
max-complexity = 10
exclude=src/apm_demos/test/*
```

## Recap
- isort, black and mypy config from `myproject.toml`
- flake8 config with its on file `.flake8`
- isort must be config with black profile

!!! tip "disabled pre-commit"
  ```
  git commit --no-verify
  ```

### .pre-commit-config.yaml

```yaml
files: src/apm_demos
repos:
  - repo: https://github.com/PyCQA/isort
    rev: 5.12.0
    hooks:
      - id: isort
  - repo: https://github.com/psf/black
    rev: 23.3.0
    hooks:
      - id: black
        language_version: python3.10
  - repo: https://github.com/pre-commit/mirrors-mypy
    rev: v0.942
    hooks:
      - id: mypy
  - repo: https://github.com/PyCQA/flake8
    rev: 6.0.0
    hooks:
      - id: flake8
```
     
---

## Reference
- [pre-commit schema](https://json.schemastore.org/pre-commit-config.json)


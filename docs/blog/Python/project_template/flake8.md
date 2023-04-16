---
tags:
    - python
    - project
    - template
    - flake8
    - linter
    - git-hook
---
# Python project template - flake8
Flake8 acts as a linter and checking cod against code styling given by PEP8
Flake8 is a wrapper which verifies pep8, pyflakes, and circular complexity.

## install

```
python -m pip install flake8
```

## config

Flake8 supports storing its configuration in your project in one of `setup.cfg`, `tox.ini`, or `.flake8`.

```ini title=".flake8"
[flake8]
extend-ignore = E121
max-line-length = 120
exclude = .git,__pycache__,docs/source/conf.py,old,build,dist

```

[Error / Violation Codes](https://flake8.pycqa.org/en/latest/user/error-codes.html)

## demo
- check code from command line
- For demoing add config file to ignore `F841`
- Run again
- 
```python title="code example"
class Demo:
    def demo_method(self):
        x = 2  # pylint: disable=unused-variable
```

```bash title="run manual"
~/.local/bin/flake8 pylint_demo.py
pylint_demo.py:3:9: F841 local variable 'x' is assigned to but never used
```

### Add flake8 config

```ini title=".flake8"
[flake8]
extend-ignore = F841
exclude = .git,__pycache__,docs/source/conf.py,old,build,dist
max-line-length = 120
```

     

---

## git hook
- Config flake8 as `local` repo
  
```yaml
repos:
  - repo: local
    hooks:
      - id: flake8
        name: flake8
        entry: /home/user/.local/bin/flake8
        language: system
        types: [python]
        args: 
            [
            "--config=.flake8"
            ]
        

```

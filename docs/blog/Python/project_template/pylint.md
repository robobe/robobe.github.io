---
title: Python project template - pylint
description: pylint
date: "2022-05-06"
banner: images/python.png
tags:
    - python
    - project
    - template
    - pylint
---

Linting is the automated source code checking for programmatic and stylistic errors.   
A lint tool is a basic static code analyzer

`pylint` is default VSCode linter and it enable by default

```bash title="install"
pip install pylint
```

```json title="vscode settings"
"python.linting.pylintEnabled": true
```

## Control

```init title="create .pylintrc"
pylint --generate-rcfile
```

### Disable messages

```init
[MASTER]
disable=
    C0114, # (missing-module-docstring)
    C0115,  # (missing-class-docstring)
```
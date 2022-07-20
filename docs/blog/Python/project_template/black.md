---
title: Python project template - black and isort
description: using black and isort
date: "2022-24-05"
banner: images/python.png
tags:
    - python
    - project
    - template
    - black
    - isort
---


### VSCode settings

```json title="settings.json"
{
    "editor.formatOnSave": true,
    "[python]": {
        "editor.codeActionsOnSave": {
            "source.organizeImports": true
        }
    },
    "python.formatting.provider": "black",
}
```



### black and isort args/settings

```toml title="pyproject.toml"
[tool.black]
line-length = 99 # override black's default line-length
exclude = '''
/(
    \.git
  | \.mypy_cache
  | \.tox
  | venv
  | \.venv
  | _build
  | buck-out
  | build
  | dist
)/
'''

[tool.isort]
# make it compatible with black
profile = "black" 
```
---
tags:
    - python
    - project
    - templates
    - cookiecutter
---

Cookiecutter uses `Jinja2` templating system

## cookiecutter.json

### private variables
cookiecutter.json can contain private variables that user won't be required to fill.

### Conditionals

```json title="cookiecutter.json"
{
    "pytest_enabled": [
        "y",
        "n"
    ]
}
```

```title="requirements_dev.txt"
mkdocs

{%- if cookiecutter.pytest_enabled == 'y' %}
pytest
{%- endif %}

```

!!! tip "jinja remove empty line"
    Add `-` after `%` to remove jinja empty lines
    for `variable` settings `if` and `for` loops
     
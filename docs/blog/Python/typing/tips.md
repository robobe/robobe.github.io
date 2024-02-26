---
tags:
    - python
    - annotation
    - typing
    - tips
---
# PEP 484 – Type Hints

## Forward references
[pep-0484](https://peps.python.org/pep-0484/#forward-references)
When a type hint contains names that have not been defined yet, that definition may be expressed as a string literal, to be resolved later.

```python
class Tree:
    def __init__(self, left: 'Tree', right: 'Tree'):
        self.left = left
        self.right = right
```
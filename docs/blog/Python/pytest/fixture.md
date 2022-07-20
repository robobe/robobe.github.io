---
title: Pytest fixture
tags:
    - pytest
    - unittest
    - python
---

Fixture are function that help us to **arrange** the domain/ environment before test
and then help us to **clean** the mess after the test

These are accessed by test functions through arguments see demo


```python
--8<-- "/home/user/projects/py_template/tests/test_with_fixture.py"
```

!!! Tip "run pytest verbose"
    ```
    python -m pytest -s <test file>
    ```


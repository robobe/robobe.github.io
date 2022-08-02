---
title: Python snippets
tags:
    - snippets
    - cheat Sheet
---

### log 

```python title="log to console"
log = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO, format="[%(asctime)s] [%(name)s] %(levelname)s - %(message)s", datefmt='%H:%M:%S')
```


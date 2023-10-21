---
tags:
    - dataclass
    - slots
    - python
---

```python
@dataclass
class Data():
    a: int
    b: float
    c: str

obj = Data(1, 2.0, "3")

@dataclass(slots=True)
class SData():
    a: int
    b: float
    c: str
obj_s = SData(1, 2.0, "3")
```

---

## Reference
- [Speed Up Your Python classes with slot](https://doziestar.medium.com/speed-upyour-python-classes-with-slot-454e0655a816)
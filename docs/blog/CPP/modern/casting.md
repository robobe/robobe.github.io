---
title: casting
tags:
    - cpp
---

- static_cast
- reinterpret_cast
- dynamic_cast


## static_cast

- Convert type of variable at compile time
- Rarely need to be use explicit
- Drive class can be upcast to its base class

```cpp
static_cast<NewType>(variable)
```

---

## reinterpret_cast
- Reinterpret the bytes of a variable as another type
- Mostly used when writing binary data

```cpp
reinterpret_cast<NewType>(variable)
```

---

## dynamic_cast
Google-style - AVOID
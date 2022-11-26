---
title: Add custom level
tags:
    - python
    - logging
---

Python has six log levels with each one assigned a specific integer indicating the severity of the log:

- NOTSET=0
- DEBUG=10
- INFO=20
- WARN=30
- ERROR=40
- CRITICAL=50

using `setLevel` method on `logging` and `handlers` we can control/filter the output logging severity  

---

## Demo

```python title="custom_level.py" linenums="1" hl_lines="5 6 24"
--8<-- "examples/python/python/log_demos/custom_level.py"
```

```title="Result" linenums="1" hl_lines="2"
2022-11-26 06:44:19,616 - mylogger - INFO -this is info
2022-11-26 06:44:19,616 - mylogger - CLIENT -this is client
2022-11-26 06:44:19,616 - mylogger - WARNING -this is warning
2022-11-26 06:44:19,616 - mylogger - ERROR -this is error
2022-11-26 06:44:19,616 - mylogger - CRITICAL -this is critical
```

!!! note "setLevel"
    The above example `setLevel` method set to severity output to `Info`  
    No `DEBUG` logging is "print"
     
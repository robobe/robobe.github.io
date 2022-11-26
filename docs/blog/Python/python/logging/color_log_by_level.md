---
title: Color log
tags:
    - python
    - logging
---

## Logging Formatting
Formatter enrich the log message by adding more information like: time, python file, logger name and more

```python title="simple_formatter.py"
import logging

FMT = "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
logging.basicConfig(format=FMT, level=logging.INFO)

log = logging.getLogger("demo")
log.info("formatter example")
```

```bash title="result"
2022-11-26 07:10:35,660 - demo - INFO - formatter example
```

| name      | format        | desc                                                                                 |
| --------- | ------------- | ------------------------------------------------------------------------------------ |
| asctime   | %(asctime)s   | time when the LogRecord was created                                                  |
| name      | %(name)s      | Name of the logger used to log the call.                                             |
| levelname | %(levelname)s | Text logging level for the message ('DEBUG', 'INFO', 'WARNING', 'ERROR', 'CRITICAL') |
| message   | %(message)s   | The logged message, computed as msg % args                                           |


!!! tip 
     [more log record attribute](https://docs.python.org/3/library/logging.html#logrecord-attributes)

---

## Custom formatter

Extend the `logging.Formatter` class and override the `format` method

- Color formatter
- Custom Level


```python title="logging/color.py" linenums="1" hl_lines="5 54"
--8<-- "examples/python/python/log_demos/color.py"
```

![](images/log_colors.png)
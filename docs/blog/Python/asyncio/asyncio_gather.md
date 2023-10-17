---
tags:
    - python
    - asyncio
---

# Asyncio gather

`asyncio.gather()` function to run multiple asynchronous operations.

```python
gather(*aws, return_exceptions=False) -> Future[tuple[()]]
```

## Simple Demo
The following example uses the `asyncio.gather()` to run two asynchronous operations and displays the results:

```python
import asyncio

async def do_work(delay, work):
    await asyncio.sleep(delay)
    return work

async def main():
    a, b = await asyncio.gather(
        do_work(2, "hello"),
        do_work(2, "world")
    )
    

    print(f"{a}, {b}")
asyncio.run(main())
```

---

## Demo: task with exception

```python title="exception" linenums="1" hl_lines="15"
import asyncio

async def do_exception(delay):
    await asyncio.sleep(delay)
    raise Exception("API failed")

async def do_work(delay, work):
    await asyncio.sleep(delay)
    return work

async def main():
    a, b = await asyncio.gather(
        do_work(2, "hello"),
        do_exception(1),
        return_exceptions=True
    )
    
    print(type(a))
    print(type(b))
    print(f"{a}, {b}")
asyncio.run(main())
```

```bash
<class 'str'>
<class 'Exception'>
hello, API failed
```

Set return_exceptions to True to allow errors to be returned as results.

---

## Reference
- [Python asyncio.gather()](https://www.pythontutorial.net/python-concurrency/python-asyncio-gather/)
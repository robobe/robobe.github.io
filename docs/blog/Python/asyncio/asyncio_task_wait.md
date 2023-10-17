---
tags:
    - python
    - asyncio
    - 
---

# Wait for task

Wait task to complete with timeout
When a timeout occurs the `wait_for` function  cancel the task and raise `asyncio.Timeout` exception


```python linenums="1" hl_lines="1"
import asyncio
import logging
import time

async def wait_func(text, delay=2):
    await asyncio.sleep(delay)


async def main():
    task1 = asyncio.create_task(wait_func("task1", delay=5))

    try:
        await asyncio.wait_for(task1, timeout=2)
    except asyncio.TimeoutError:
        print("task cancelled by timeout")


asyncio.run(main())

```

---

# Wait

The asyncio.wait() function runs an iterable of awaitables objects and blocks until a specified condition.

| Condition  | Description  |
|---|---|
| FIRST_COMPLETED  | Return when all awaitables are complete or canceled.  |
| FIRST_EXCEPTION  | Return when any awaitable is raising an exception  |
| ALL_COMPLETED  | Return when **all** awaitables are complete or cancelled.  |

```python
done, pending = asyncio.wait(aws, *, timeout=None, return_when=ALL_COMPLETED)
```

- `done`: **Set** of a awaitables that are done
- `pending`: **Set** of a awaitables that are pending


---

## Demo

```python title="ALL_COMPLETED"
import asyncio
import logging
import time

logging.basicConfig(format="%(asctime)s %(message)s", level=logging.DEBUG, datefmt='%H:%M:%S')
log = logging.getLogger(__name__)

async def wait_func(text, delay=2):
    log.info(f"start {text}")
    await asyncio.sleep(delay)
    log.info(f"end {text}")


async def main():
    start = time.perf_counter()
    task1 = asyncio.create_task(wait_func("task1", delay=3))
    task2 = asyncio.create_task(wait_func("task2", delay=2))
    task3 = asyncio.create_task(wait_func("task3", delay=5))

    done, pending = await asyncio.wait([task1, task2, task3], return_when=asyncio.ALL_COMPLETED)
    
    end = time.perf_counter()
    log.info(f'It took {round(end-start,0)} second(s) to complete.')


asyncio.run(main())
```

```bash
10:26:39 start task1
10:26:39 start task2
10:26:39 start task3
10:26:41 end task2
10:26:42 end task1
10:26:44 end task3
10:26:44 It took 5.0 second(s) to complete.
```

### xxx

```
```


---

## Reference
- [asyncio.wait](https://www.pythontutorial.net/python-concurrency/python-asyncio-wait/)


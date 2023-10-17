---
tags:
    - python
    - asyncio
    - task
---

# Scheduler coroutine

A task is a wrapper of a coroutine that schedules the coroutine to run on the event loop as soon as possible.

![](images/event_loop_task.png)
[copyright](https://medium.com/dev-bits/a-minimalistic-guide-for-understanding-asyncio-in-python-52c436c244ea)


```python
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
    
    task1 = asyncio.create_task(wait_func("task1"))
    task2 = asyncio.create_task(wait_func("task2"))

    await task1
    await task2

    end = time.perf_counter()
    log.info(f'It took {round(end-start,0)} second(s) to complete.')

asyncio.run(main())
```

```bash
05:54:25 start task1
05:54:25 start task2
05:54:27 end task1
05:54:27 end task2
05:54:27 It took 2.0 second(s) to complete.
```

---

## Task cancel
Cancel running task

!!! note ""
    await a canceled task will raise a CancelledError exception.
     
```python linenums="1" hl_lines="17 21"
import asyncio


async def wait_func(delay=4):
    await asyncio.sleep(delay)


async def main():
    task = asyncio.create_task(wait_func())

    time_elapsed = 0
    while not task.done():
        await asyncio.sleep(1)
        time_elapsed += 1
        if time_elapsed == 2:
            print("task cancel request")
            task.cancel()
            break
    try:
        await task
    except asyncio.CancelledError:
        print("Task has been cancelled by user")


asyncio.run(main())

```
     
---

## Reference
- [Cancelling Tasks](https://www.pythontutorial.net/python-concurrency/python-cancel-tasks/)
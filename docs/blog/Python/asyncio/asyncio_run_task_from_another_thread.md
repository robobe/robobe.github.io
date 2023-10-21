---
tags:
    - python
    - asyncio
    - thread
---

# Run task from another thread

Using `run_coroutine_threadsafe` to submit a coroutine to a given event loop from another thread.

```
asyncio.run_coroutine_threadsafe(coro, loop)
```

## Simple Demo

```python
import asyncio
from threading import Thread
import time

async def main():
    await asyncio.sleep(5)
    print("end main, worker shutdown")

async def handler_task(counter):
    print(f"current counter: {counter}")

def worker(main_loop):
    counter = 0
    while True:
        asyncio.run_coroutine_threadsafe(handler_task(counter), main_loop)
        counter += 1
        time.sleep(1/2)

loop = asyncio.get_event_loop()
t_worker = Thread(target=worker, args=(loop, ), daemon=True, name="worker")
t_worker.start()
task = loop.create_task(main())
loop.run_until_complete(task)

```

---

## Reference
- [Python asyncio.run_coroutine_threadsafe() function](https://www.slingacademy.com/article/python-asyncio-run_coroutine_threadsafe/)
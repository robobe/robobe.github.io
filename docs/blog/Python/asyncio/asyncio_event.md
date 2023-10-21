---
tags:
    - python
    - asyncio
    - event
---

# Asyncio event
Notify between coroutines

## Demo

```python title="event_demo.py" linenums="1" hl_lines="5 11 14"
import asyncio

async def to_be_notify(event: asyncio.Event):
    print("task wait to be notify")
    await event.wait()
    print("task notified")

async def notify_task(event: asyncio.Event):
    await asyncio.sleep(3)
    print("notify task set")
    event.set()

async def main():
    event = asyncio.Event()
    co_1 = to_be_notify(event)
    co_2 = notify_task(event)
    await asyncio.gather(co_1, co_2)


asyncio.run(main())

```


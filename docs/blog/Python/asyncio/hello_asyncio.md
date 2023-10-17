---
tags:
    - python
    - asyncio
---

Asyncio achieves concurrency through the use of coroutines. **Coroutines** are functions that can be **paused** and **resumed** at specific points during their execution. This allows multiple coroutines to run concurrently within a single thread.

When programming, asynchronous means that the action is requested/ schedule, although **not performed at the time of the request**. It is performed later.

- **coroutine** is a function that can be suspended and resumed.
  
- **Future**: A handle on an asynchronous function call allowing the status of the call to be checked and results to be retrieved.


- **Asynchronous Task**: Used to refer to the aggregate of an asynchronous function call and resulting future


### coroutine
A coroutine is a regular function with the ability to pause and resume its execution 

to mark function as coroutine we add the `async` keyword before def statement.
To pause coroutine execution we use the `await` keyword.


#### async
async mark function as coroutine
coroutine execute on event loop 
there other asyncio method to register/schedule the coroutine on event loop 

```python title="coroutine"
async def hello() -> int:
    pass

co = hello()
print(co)
```

```bash
<coroutine object hello at 0x7f5058f6b530>
sys:1: RuntimeWarning: coroutine 'hello' was never awaited
```

#### await

The await keyword pauses the execution of a coroutine

```
result = await my_coroutine()
```

The `await` keyword causes the my_coroutine() to execute, waits for the code to be completed, and returns a result.

---

## asyncio app

```python linenums="1" hl_lines="8 10 13"
import asyncio
import logging

logging.basicConfig(format="[%(levelname)s] %(asctime)s %(message)s", level=logging.DEBUG)
log = logging.getLogger(__name__)

async def main():
    log.info("Starting coroutine")
    await asyncio.sleep(2)
    log.info("Coroutine finished")

asyncio.run(main())
```

The asyncio.run() function used to run a coroutine in an event loop. This function: 
- creates an event loop, 
- runs the coroutine in the event loop, 
- closes the event loop when the coroutine is complete.

--- 

## Reference
- [python tutorial asyncio](https://www.pythontutorial.net/python-concurrency/python-event-loop/)
[Guide to Concurrency in Python with Asyncio](https://www.integralist.co.uk/posts/python-asyncio/#futures)

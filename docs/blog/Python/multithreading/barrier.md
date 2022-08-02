---
title: Multithreading - barrier
tags:
    - multithreading
---

Barrier objects are used to wait for a fixed number of thread to complete execution before any particular thread can proceed forward with is execution

### Demo
- sleep function simulate to work load
- init barrier for 2 threads
- thread `t2` wait for `t1` on barrier
- when `t1` reach the barrier both threads `t1, t2` can continue


```python title="barrier" linenums="1" hl_lines="17"
import threading
import logging
import time

log = logging.getLogger(__name__)
logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s] [%(name)s] %(levelname)s - %(message)s",
    datefmt="%H:%M:%S",
)


def job(name: str, wait_time: int) -> None:
    log.info(f"Thread {name} running")
    time.sleep(wait_time)
    log.info(f"Thread {name} wait on barrier")
    barrier.wait()
    log.info(f"Thread {name} continue running")


barrier = threading.Barrier(2)
t1 = threading.Thread(target=job, args=("t1", 5))
t2 = threading.Thread(target=job, args=("t2", 3))
t1.start()
t2.start()
```

#### result
```
[07:17:58] [__main__] INFO - Thread t1 running
[07:17:58] [__main__] INFO - Thread t2 running
[07:18:01] [__main__] INFO - Thread t2 wait on barrier
[07:18:03] [__main__] INFO - Thread t1 wait on barrier
[07:18:03] [__main__] INFO - Thread t1 continue running
[07:18:03] [__main__] INFO - Thread t2 continue running
```

# Reference
- [ Python's Thread Barriers in 8 mins ](https://youtu.be/3Y6l76AS4l4)




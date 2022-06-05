import asyncio
import time
import threading
import logging

logging.basicConfig(format="[%(levelname)s] %(asctime)s %(message)s", level=logging.DEBUG)
log = logging.getLogger(__name__)

start = None

def fetch(with_time: int)-> None:
    log.info(threading.current_thread().name)
    time.sleep(with_time)

async def do_work(delay, work):
    await asyncio.sleep(delay)
    log.info(work)


def callback(future: asyncio.Future) -> None:
    end = time.time()
    log.info(f"callback run after: {end-start}")

async def main():
    log.info(threading.current_thread().name)
    my_loop = asyncio.get_event_loop()
    asyncio.create_task(do_work(1, "hello"))
    asyncio.create_task(do_work(2, "hello2"))
    future = my_loop.run_in_executor(None, fetch, 3)
    global start
    start = time.time()
    future.add_done_callback(callback)
    await future
    end = time.time()
    log.info(f"continue run after await time {end-start}")
    await asyncio.create_task(do_work(1, "hello end "))

asyncio.run(main())

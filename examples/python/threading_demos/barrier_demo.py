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
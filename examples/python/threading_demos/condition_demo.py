import time
from threading import *
import logging


log = logging.getLogger(__name__)
logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s] [%(name)s] %(levelname)s - %(message)s",
    datefmt="%H:%M:%S",
)


class appointment:
    def subscriber(self):
        while True:
            condition_object.acquire()
            log.info("wait")
            condition_object.wait()  # Thread is in waiting state
            log.info("run job")

            condition_object.release()

    def publisher(self):
        condition_object.acquire()
        for _ in range(5):
          
          time.sleep(1)
          log.info("notfy")
          condition_object.notify()
          condition_object.release()


condition_object = Condition()
class_obj = appointment()

T1 = Thread(target=class_obj.subscriber)

T2 = Thread(target=class_obj.publisher)

T1.start()

T2.start()

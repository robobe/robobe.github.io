---
tags:
    - python
    - zmq
    - opencv
    - numpy
---

# Pub image over ZMQ
Send image over zmq

- Capture or Generate image
- Using numpy and convert array to bytes stream using `tobytes` method
- Pub using zmq `send_pyobj`
- Receive data and convert to array `frombuffer`
- Reshape image using `reshape`


```python
import multiprocessing
import zmq
import time
import cv2
import numpy as np
from time import strftime


TOPIC = "topic"
HEIGHT = 480
WIDTH = 640
CHANNELS = 3


def pub():
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind("tcp://*:5555")
    print("Binding to port 5555")
    while True:
        blank_image = np.zeros((HEIGHT, WIDTH, CHANNELS), np.uint8)
        cv2.putText(
            blank_image,
            strftime("%H:%M:%S"),
            (10, 70),
            cv2.FONT_HERSHEY_SIMPLEX,
            2,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
        socket.send_string(TOPIC, zmq.SNDMORE)
        data = blank_image.tobytes()
        socket.send_pyobj(data)
        time.sleep(1 / 10)


def sub():
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.connect("tcp://127.0.0.1:5555")
    socket.setsockopt_string(zmq.SUBSCRIBE, TOPIC)
    while True:
        topic = socket.recv_string()
        raw = socket.recv_pyobj()
        data = np.frombuffer(raw, dtype=np.uint8)
        img = data.reshape((HEIGHT, WIDTH, CHANNELS))
        cv2.imshow("pub_sub_image", img)
        cv2.waitKey(1)


if __name__ == "__main__":
    p_pub = multiprocessing.Process(target=pub)
    p_sub = multiprocessing.Process(target=sub)
    p_sub.start()
    p_pub.start()

    p_pub.join()
    p_sub.join()

```


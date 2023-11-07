---
tags:
    - opencv
    - gstreamer
    - 
---

!!! note "gstreamer support"
    OpenCV need to build with gstreamer support

    Check support with `cv2.getBuildInformation()` function

    ```
    ...
    Video I/O:
    DC1394:                      YES (2.2.6)
    FFMPEG:                      YES
      avcodec:                   YES (58.134.100)
      avformat:                  YES (58.76.100)
      avutil:                    YES (56.70.100)
      swscale:                   YES (5.9.100)
      avresample:                NO
    GStreamer:                   YES (1.19.90)
    
    ...
    ```
     

## Demo

```python
import cv2


FPS = 10
SIZE = (640, 480)
# FourCC is a 4-byte code used to specify the video codec
FOURCC = 0 # uncompressed

cap = cv2.VideoCapture(f"videotestsrc ! video/x-raw,width=640,height=480,framerate={FPS}/1 ! videoconvert ! timeoverlay ! appsink")
out_pipe = f"appsrc ! video/x-raw,width=640,height=480,framerate={FPS}/1 ! videoconvert ! timeoverlay xpad=100 ypad=100 ! autovideosink sync=false"
out = cv2.VideoWriter(out_pipe, FOURCC, FPS, SIZE)

while True:
    ret, frame = cap.read()
    cv2.imshow("cv", frame)
    out.write(frame)
    if cv2.waitKey(1) & 0xff == ord('q'):
        break
cap.release()
out.release()
cv2.destroyAllWindows()
```

---

## Demo 2
- Process1
  - Use Gstreamer to generate video into appsink
  - Use OpenCV to capture/read from appsink
  - Use OpenCV to write the frame into `Gstreamer pipe` using `appsrc`
- Process 2
  - Capture the frame using `Gstreamer pipe` into `appsink`
  - Show the frame

```python
from multiprocessing import Process
import cv2


def send():
    cap_send = cv2.VideoCapture('videotestsrc ! video/x-raw,framerate=20/1 ! videoscale ! videoconvert ! timeoverlay ! appsink', cv2.CAP_GSTREAMER)
    out_send = cv2.VideoWriter('appsrc ! videoconvert ! x264enc tune=zerolatency bitrate=500 speed-preset=superfast ! rtph264pay ! udpsink host=127.0.0.1 port=5000',cv2.CAP_GSTREAMER,0, 20, (320,240), True)

    if not cap_send.isOpened() or not out_send.isOpened():
        print('VideoCapture or VideoWriter not opened')
        exit(0)

    while True:
        ret,frame = cap_send.read()

        if not ret:
            print('empty frame')
            break

        out_send.write(frame)

        cv2.imshow('send', frame)
        if cv2.waitKey(1)&0xFF == ord('q'):
            break

    cap_send.release()
    out_send.release()

def receive():
    cap_receive = cv2.VideoCapture('udpsrc port=5000 caps = "application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96" ! rtph264depay ! decodebin ! videoconvert ! timeoverlay ! appsink', cv2.CAP_GSTREAMER)

    if not cap_receive.isOpened():
        print('VideoCapture not opened')
        exit(0)

    while True:
        ret,frame = cap_receive.read()

        if not ret:
            print('empty frame')
            break

        cv2.imshow('receive', frame)
        if cv2.waitKey(1)&0xFF == ord('q'):
            break

    #cap_receive.release()

if __name__ == '__main__':
    s = Process(target=send)
    r = Process(target=receive)
    s.start()
    r.start()
    s.join()
    r.join()

    cv2.destroyAllWindows()
```


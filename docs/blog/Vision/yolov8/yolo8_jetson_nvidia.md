---
tags:
    - yolo
    - yolov8
    - nvidia
    - jetson
    - tensorrt
---

Run yolo using pytorch and tensortrt models

- Convert pytorch model to onnx and tensorrt

## prerequisite
- torch
- tensorrt


```
pip install pytorch
pip install tensorrt
```

```bash title="torch model (pt)"
yolo detect predict model=yolov8n.pt source='https://ultralytics.com/images/bus.jpg' show=True
yolo detect predict model=yolov8n.pt source='bus.jpg' show=True
# result
image 1/1 /home/user/projects/yolo_v8_tutorial/bus.jpg: 640x480 4 persons, 1 bus, 1 stop sign, 64.0ms
Speed: 2.8ms preprocess, 64.0ms inference, 1.0ms postprocess per image at shape (1, 3, 640, 480)
```

### Convert torch(pt) to tensorrt(engine)

```bash
yolo export model=yolov8n.pt format=engine half=True device=0
```

```bash title="tensorrt model (engine)"
yolo detect predict model=yolov8n.engine source=bus.jpg show=True
#result
image 1/1 /home/user/projects/yolo_v8_tutorial/bus.jpg: 640x640 4 persons, 1 bus, 6.7ms
Speed: 13.4ms preprocess, 6.7ms inference, 265.2ms postprocess per image at shape (1, 3, 640, 640)

```



---

- [Batch inference implementation using tensorrt#3 — batch inference using TensorRT python api](https://medium.com/@smallerNdeeper/yolov8-batch-inference-implementation-using-tensorrt-3-batch-inference-using-tensorrt-python-cf30ae10920c)
---
title: YOLOv5
description: Object Detection using YOLOv5 and OpenCV DNN in Python
date: "2022-05-09"
banner: ../images/ai.png
tags:
    - yolo
    - cuda
---

Using openCV dnn module to run yolo5 models  
Convert pytorch models to onnx model using `export.py` from yolo5 repository

# project
All the code and module are download from [LearnOpenCV]()
```
├── coco.names
├── detect.py
├── models
│   ├── yolov5n.onnx
│   └── yolov5s.onnx
└── sample.jpg
```

---

# detect code

!!! warning
    When use cuda, first image initialize the pipe between the cpu memory and gpu

!!! tip
    using dnn cuda support from opencv 4.2
    ```python
    net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
	net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
    ```

```python
--8<-- blog/examples/open_cv/dnn/yolo5/detect.py
```

### Run sample
#### small model
#### cpu

run time: 130 ms

![](images/cpu.png)

#### gpu

run time: 39 ms

![](images/gpu.png)

---

# References

- [Object Detection using YOLOv5](https://learnopencv.com/object-detection-using-yolov5-and-opencv-dnn-in-c-and-python/)
- [Object Detection using YOLOv5 github](https://github.com/spmallick/learnopencv/tree/master/Object-Detection-using-YOLOv5-and-OpenCV-DNN-in-CPP-and-Python)
- [How to use OpenCV DNN Module with NVIDIA GPUs](https://learnopencv.com/opencv-dnn-with-gpu-support/)
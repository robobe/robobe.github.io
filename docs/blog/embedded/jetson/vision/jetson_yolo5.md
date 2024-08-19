---
tags:
    - jetson
    - nvidia
    - yolov5
    - 
---

# Install yolov5 on jetson orin

- Install [cuda]
- Install [opencv]
- Install [pytorch](jetson_pytorch_cuda_support)


## yolov5

```bash
git clone https://github.com/ultralytics/yolov5
cd yolov5

# install requirements
python3 -m pip install --no-cache -r requirements.txt
```


### minimal test

```bash
import torch

# Model
model = torch.hub.load('ultralytics/yolov5', 'yolov5s')  # or yolov5n - yolov5x6, custom

# Images
img = 'https://ultralytics.com/images/zidane.jpg'  # or file, Path, PIL, OpenCV, numpy, list

# Inference
results = model(img)

# Results
results.print()  # or .show(), .save(), .crop(), .pandas(), etc.
```

---

[yolo tensor rt](https://github.com/mailrocketsystems/JetsonYolov5)
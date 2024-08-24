---
tags:
    - jetson
    - nvidia
    - yolov5
    - 
---
# yolov5
Run yolo with torch and tensorrt

## Install yolov5 on jetson orin

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

## inference using TensorRT

```bash title="check tensorrt version"
>>> import tensorrt
>>> tensorrt.__version__
'8.6.2'

```

### convert
```
python export.py --weights yolov5s.pt --data data/coco128.yaml --include engine --device 0 
```

### usage
#### onnx

[Quick Start Guide: NVIDIA Jetson with Ultralytics YOLOv8](https://github.com/ultralytics/ultralytics/blob/main/docs/en/guides/nvidia-jetson.md)
```
wget https://nvidia.box.com/shared/static/48dtuob7meiw6ebgfsfqakc9vse62sg4.whl -O onnxruntime_gpu-1.18.0-cp310-cp310-linux_aarch64.whl
pip install onnxruntime_gpu-1.18.0-cp310-cp310-linux_aarch64.whl
```
---

## Reference

[yolo tensor rt](https://github.com/mailrocketsystems/JetsonYolov5)
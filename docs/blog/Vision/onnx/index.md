---
tags:
    - onnx
    - inference
---
Using onnx runtime as inference engine

```bash
pip install onnxruntime
pip install opencv-python
```

## Demo 
Using `MINST` model to run onnx inference


```python
import json
import numpy as np
import cv2
import onnxruntime
import pathlib

	
base_dir = pathlib.Path("/home/user/projects/yolo_v8_tutorial/src/onnx_demo/mnist")
model=base_dir.joinpath("model.onnx").as_posix()
image_path= base_dir.joinpath("img_98.jpg").as_posix()
session = onnxruntime.InferenceSession(model, None)
input_name = session.get_inputs()[0].name
output_name = session.get_outputs()[0].name

#Preprocess the image
img = cv2.imread(image_path)
img = np.dot(img[...,:3], [0.299, 0.587, 0.114])
img = cv2.resize(img, dsize=(28, 28), interpolation=cv2.INTER_AREA)
img.resize((1, 1, 28, 28))

data = json.dumps({'data': img.tolist()})
data = np.array(json.loads(data)['data']).astype('float32')

# Inderence
result = session.run([output_name], {input_name: data})

# Postprocess
prediction=int(np.argmax(np.array(result).squeeze(), axis=0))
print(prediction)
```

---

## onnxruntime-gpu
Same API just on GPU

!!! note
    Remove `onnxruntime` if installed


```

pip uninstall onnxruntime
pip install onnxruntime-gpu

```

```
>> import onnxruntime as ort
>> ort.get_device()
'GPU'
```

---

## Reference
- [Using a Pre-Trained ONNX Model for Inferencing](https://thenewstack.io/tutorial-using-a-pre-trained-onnx-model-for-inferencing/)
- []()

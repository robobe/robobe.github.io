---
tags:
    - yolo
    - ultralytics
    - docker
    - container toolkit
---

- [Docker](#docker)
  - [Pull image](#pull-image)
  - [usage](#usage)
- [VSCode dev-container](#vscode-dev-container)
  - [dev container with gui](#dev-container-with-gui)
- [yolo cli](#yolo-cli)
  - [predict](#predict)
- [yolo python simple](#yolo-python-simple)
- [yolo and opencv](#yolo-and-opencv)


## Docker
Use docker image from ultralytics and vscode to develop yolo project
[Docker Quickstart Guide for Ultralytics](https://docs.ultralytics.com/guides/docker-quickstart/)

!!! tip nvidia container toolkit
    NVIDIA Container Toolkit enables users to build and run GPU-accelerated containers.
    - [NVIDIA container toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/index.html)
    ### install
    - [NVIDIA container toolkit install](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)

    ### config
    modify `/etc/docker/daemon.json` using `nvidia-ctk`
    ```bash
    sudo nvidia-ctk runtime configure --runtime=docker
    ```

    ### usage
    [](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/sample-workload.html)
    ```bash
    sudo docker run --rm --runtime=nvidia --gpus all ubuntu nvidia-smi
    ```
     

### Pull image
Check image types at [DockerHUB](https://hub.docker.com/r/ultralytics/ultralytics/tags)


```bash
sudo docker pull ultralytics/ultralytics:latest
```

### usage

```bash
docker run -it --ipc=host --runtime=nvidia --gpus all ultralytics/ultralytics:latest /bin/bash
```
    

---

## VSCode dev-container

- Add user `user` to image

```Dockerfile
FROM ultralytics/ultralytics

ARG USERNAME=user
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create a non-root user
RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  # Add sudo support for the non-root user
  && apt-get update \
  && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && rm -rf /var/lib/apt/lists/*

RUN  && apt-get update && apt-get install -y \
    #image viewer
    feh \ 
  && rm -rf /var/lib/apt/lists/*
```

<details>
<summary>minimal devcontainer.json</summary>

```json name="devcontainer.json" title="minimal devcontainer file"
{
    "name": "yolo",
    "build": {  
        "dockerfile": "Dockerfile",
        "context": ".."
    },
    "runArgs": [
        "--name=yolo-dev",
        "--hostname=yolo",
        "--runtime=nvidia",
        "--network=host",
        "--gpus=all",
        "--ipc=host"
    ],
    "remoteUser": "user"
}

```

</details>

---

### dev container with gui 

Dev container with x11 support

```json
{
    "name": "yolo",
    "build": {  
        "dockerfile": "Dockerfile",
        "context": ".."
    },
    "runArgs": [
        "--name=yolo-dev",
        "--hostname=yolo",
        "--runtime=nvidia",
        "--network=host",
        "--gpus=all",
        "--ipc=host",
        "--device=/dev/dri:/dev/dri"
    ],
    "remoteUser": "user",
    "containerEnv": {
        "DISPLAY": "${localEnv:DISPLAY}",
        "QT_X11_NO_MITSHM": "1"
    },
    "mounts": [
        "source=/tmp/.X11-unix,target=/tmp/.X11-unix,type=bind,consistency=cached"
    ],
    "customizations": {
        "vscode": {
            "extensions": [
                "ms-python.python",
                "redhat.vscode-yaml"
            ],
            "settings": {

            }
        }
    }
}
```

---

## yolo cli
### predict
[ultralytics predict](https://docs.ultralytics.com/modes/predict/)
Predict mode is used for making predictions using a trained YOLOv8 model on new images or videos

```bash
yolo predict imgsz=640 conf=0.25 \
model=models/yolov8n.pt \
source=images \
project=/tmp/results \
name=my_results
```

- model: model path
- source: image folder path
- project: result output folder
- name: project name in output folder
- conf: The conf argument sets the confidence threshold for filtering predictions. (0-1)
- imgsz: Resizes the **input images** to a specific size before inference. Larger sizes improve accuracy but slow down inference.


## yolo python simple

```python
from ultralytics import YOLO

model = YOLO("yolov8n.pt")
model.predict(
    source="/workspaces/yolo_tutorial/yolo_tutorial/images",
    imgsz=640,              # Image size for inference
    conf=0.25,               # Confidence threshold
    save=True,              # Save the prediction images to disk
    project='/tmp/results',         # Project directory where results will be saved
    name='custom_run'
)
```

## yolo and opencv

```python
from ultralytics import YOLO
import cv2
model = YOLO("models/yolov8n.pt")

img = cv2.imread('images/bus.jpg')
results = model.predict(source=img, conf=0.25, imgsz=640)
for result in results:
    annotated_frame = results[0].plot()

    cv2.imshow('YOLOv8 Detection', annotated_frame)
    k = cv2.waitKey(0)
    if k==27:
        break

cv2.destroyAllWindows()
```

!!! tip using gpu

    ```python
    results = model.predict(source=img, conf=0.25, imgsz=640, device="cuda")
    device = model.device  # This will return either 'cuda' or 'cpu'
    print(f"YOLOv8 is using: {device}")
    ```
     
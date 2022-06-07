---
title: Build OpenCV with CUDA
description: Build OpenCV with CUDA
date: "2022-06-01"
banner: ../images/logo.jpeg
tags:
    - opencv
    - build
    - cuda
---
Build OpenCV on x64 ubuntu machine

## Prerequisites
- install nvidia driver [...](https://robobe.github.io/blog/AI/pc_nvidia_install/#install-driver)
- install cudnn [...](https://robobe.github.io/blog/AI/pc_nvidia_install/#cudnn)
- install cmake gui
- install all opencv dependencies
  
```bash
sudo apt install cmake-qt-gui
```

```bash title="install dependencies"
sudo apt-get update
sudo apt-get upgrade
sudo apt-get install build-essential cmake unzip pkg-config
sudo apt-get install libjpeg-dev libpng-dev libtiff-dev
sudo apt-get install libavcodec-dev libavformat-dev libswscale-dev
sudo apt-get install libv4l-dev libxvidcore-dev libx264-dev
sudo apt-get install libgtk-3-dev
sudo apt-get install libblas-dev liblapack-dev gfortran
sudo apt-get install python3-dev
```


### Download source

```bash
wget https://github.com/opencv/opencv/archive/4.5.5.zip
wget https://github.com/opencv/opencv_contrib/archive/4.5.5.zip

# unzip
```

---

## build process

- set all option
- run configure 
- run generate
- run make -j 8 from/on build folder
- check

## config

![](images/cmakesetup.png)

- Select source and build folder

![](images/source_build_group.png)

- config python

![](images/config_python.png)

- config cuda
![](images/with_cuda.png)

- gpu arch
[wikipedia cuda gpu support](https://en.wikipedia.org/wiki/CUDA#GPUs_supported)
![https://en.wikipedia.org/wiki/CUDA#GPUs_supported](images/gpu_arch.png)


![](images/cuda_config.png)

- dnn

![](images/dnn_cuda.png)

- blas

![](images/cublas.png)

- fast math

![](images/fast_math.png)

- contrib
  
![](images/contrib.png)

- pkgconfig

![](images/pkg_config.png)


### config result
- show `my changes`

```
Commandline options:
-DOPENCV_EXTRA_MODULES_PATH:PATH="/home/user/opencv/opencv_contrib-4.5.5/modules" \
-DOPENCV_GENERATE_PKGCONFIG:BOOL="1" \
-DWITH_CUDA:BOOL="1" \
-DCUDA_ARCH_BIN:STRING="7.5" \
-DOPENCV_DNN_CUDA:BOOL="1" \
-DENABLE_FAST_MATH:BOOL="1" \
-DPYTHON3_EXECUTABLE:FILEPATH="/usr/bin/python3" -DBUILD_opencv_world:BOOL="1" \
-DCUDA_FAST_MATH:BOOL="1" 


Cache file:
OPENCV_EXTRA_MODULES_PATH:PATH=/home/user/opencv/opencv_contrib-4.5.5/modules
OPENCV_GENERATE_PKGCONFIG:BOOL=1
WITH_CUDA:BOOL=1
CUDA_ARCH_BIN:STRING=7.5
OPENCV_DNN_CUDA:BOOL=1
ENABLE_FAST_MATH:BOOL=1
PYTHON3_EXECUTABLE:FILEPATH=/usr/bin/python3
BUILD_opencv_world:BOOL=1
CUDA_FAST_MATH:BOOL=1

```

---



## Check
- create new virtual env and activate
- run setup.py from `build/python_loader` folder


```python
import cv2
cv2.__version__
cv2.cuda.getCudaEnabledDeviceCount()

```
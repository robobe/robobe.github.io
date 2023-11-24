---
tags:
    - opencv
    - build
    - cuda
    - source
---

base on [build from source](build_openc_cv_form_source.md)


## cuda
!!! tip "cuda gcc compatibility"
     ubuntu 22.04 with cuda 11.5 has compatibility issue with gcc-11
## gcc

```bash
sudo apt install gcc-10 g++-10
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-10 10
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-10 10
sudo update-alternatives --config gcc
sudo update-alternatives --config g++ 
```
### check if install
[How to find the NVIDIA cuda version](https://www.cyberciti.biz/faq/how-to-find-the-nvidia-cuda-version/)
```bash
nvidia-smi
```

```bash
nvcc --version
#
nvcc: NVIDIA (R) Cuda compiler driver
Copyright (c) 2005-2021 NVIDIA Corporation
Built on Thu_Nov_18_09:45:30_PST_2021
Cuda compilation tools, release 11.5, V11.5.119
Build cuda_11.5.r11.5/compiler.30672275_0

```

```
-D WITH_CUDA=ON \
-D BUILD_opencv_cudacodec=OFF \
```

---

## cudnn
NVIDIA CUDA Deep Neural Network (cuDNN) is a GPU-accelerated library of primitives for deep neural networks. cuDNN is built on top of the CUDA framework

### check version
```bash
cat /usr/include/x86_64-linux-gnu/cudnn_v*.h | grep CUDNN_MAJOR -A 2

```

```
-D WITH_CUDNN=ON \
-D OPENCV_DNN_CUDA=ON \
-D CUDA_ARCH_BIN=7.5 \
```

### final 

```
WORKING_DIR=/home/user/opencv
OUTPUT=$(nvcc --list-gpu-arch | tail -1)
CUDA_ARCH_BIN=${OUTPUT: -2:-1}.${OUTPUT: -1}
echo CUDA Compute Capability is "${CUDA_ARCH_BIN}"

cmake \
-D CMAKE_BUILD_TYPE=RELEASE  \
-D CMAKE_INSTALL_PREFIX=/usr/local \
-D WITH_FFMPEG=OFF \
-D BUILD_JAVA=OFF \
-D BUILD_opencv_python3=ON \
-D WITH_GSTREAMER=ON \
-D OPENCV_GENERATE_PKGCONFIG=ON \
-D OPENCV_PC_FILE_NAME=opencv.pc \
-D INSTALL_PYTHON_EXAMPLES=OFF \
-D INSTALL_C_EXAMPLES=OFF \
-D BUILD_EXAMPLES=OFF \
-D OpenCV_cudev=OFF \
-D WITH_CUDA=ON \
-D BUILD_opencv_cudacodec=OFF \
-D WITH_CUDNN=ON \
-D OPENCV_DNN_CUDA=ON \
-D CUDA_ARCH_BIN="${CUDA_ARCH_BIN}" \
-D PYTHON3_EXECUTABLE=`python -c "import sys; print(sys.executable)"` \
-D PYTHON3_INCLUDE_DIR=/usr/include/python3.10 \
-D PYTHON3_NUMPY_INCLUDE_DIRS=`python3 -c "import numpy; print(numpy.get_include())"` \
-D PYTHON3_PACKAGES_PATH=`python3 -c "import sysconfig; print(sysconfig.get_paths()['purelib'])"` \
-D OPENCV_PYTHON3_INSTALL_PATH=`python3 -c "import sysconfig; print(sysconfig.get_paths()['purelib'])"` \
-D OPENCV_EXTRA_MODULES_PATH=${WORKING_DIR}/opencv_contrib/modules \
..

```


```bash title="config output for cuda"
--   NVIDIA CUDA:                   YES (ver 11.5, CUFFT CUBLAS)
--     NVIDIA GPU arch:             87
--     NVIDIA PTX archs:
-- 
--   cuDNN:                         YES (ver 8.4.0)

```

!!! tip "Python.h missing"
    Add environment variable `CPATH`
    ```bash
    export CPATH=/usr/include/python3.5m:$CPATH
    ```

---
     
## Test

```python
import cv2
    count = cv2.cuda.getCudaEnabledDeviceCount()
print(count)

```
---

## Reference
- [OpenCV configuration options reference](https://docs.opencv.org/4.x/db/d05/tutorial_config_reference.html)
- [Install OpenCV 4.5.1 with CUDA 11.1 in Ubuntu 20.04 LTS](https://gahan9.medium.com/install-opencv-4-5-1-with-cuda-11-1-in-ubuntu-20-04-lts-4af667287d9d)
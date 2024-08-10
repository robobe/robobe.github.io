---
tags:
    - nx
    - opencv
    - cuda
    - build

---

# Build OpenCV from source with cuda support on Orin NX

Base on Jetpack 6.0 cuda installed 

  
## Build steeps
```bash title="pre install"
sudo apt-get update
sudo apt-get install -y build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
sudo apt-get install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
sudo apt-get install -y python3.10-dev python3-numpy
sudo apt-get install -y libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev
sudo apt-get install -y libv4l-dev v4l-utils qv4l2 v4l2ucp
sudo apt-get install -y curl
```

```bash title="opencv download and extract"
version="4.10.0"
folder="workspace"
mkdir $folder
cd ${folder}
curl -L https://github.com/opencv/opencv/archive/${version}.zip -o opencv-${version}.zip
curl -L https://github.com/opencv/opencv_contrib/archive/${version}.zip -o opencv_contrib-${version}.zip
unzip opencv-${version}.zip
unzip opencv_contrib-${version}.zip
rm opencv-${version}.zip opencv_contrib-${version}.zip
cd opencv-${version}/
```

```bash title="cmake"
mkdir release
cd release/

cmake \
-D WITH_CUDA=ON \
-D WITH_CUDNN=ON \
-D CUDA_ARCH_BIN="7.2,8.7" \
-D CUDA_ARCH_PTX="" \
-D OPENCV_GENERATE_PKGCONFIG=ON \
-D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-${version}/modules \
-D WITH_GSTREAMER=ON \
-D WITH_LIBV4L=ON \
-D BUILD_opencv_python3=ON \
-D BUILD_TESTS=OFF \
-D BUILD_PERF_TESTS=OFF \
-D BUILD_EXAMPLES=OFF \
-D CMAKE_BUILD_TYPE=RELEASE \
-D CMAKE_INSTALL_PREFIX=/usr/local \
-D CPACK_BINARY_DEB=ON ..

make -j$(nproc)
```

```bash title="make install and config bashrc"
sudo make install
echo 'export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
echo 'export PYTHONPATH=/usr/local/lib/python3.8/site-packages/:$PYTHONPATH' >> ~/.bashrc
source ~/.bashrc
```

### check

```python
import cv2
>>> cv2.__version__
'4.10.0'
# getBuildInformation
>>> print(cv2.getBuildInformation())
...
  NVIDIA CUDA:                   YES (ver 12.2, CUFFT CUBLAS)
    NVIDIA GPU arch:             72 87
    NVIDIA PTX archs:

  cuDNN:                         YES (ver 8.9.4)


#getCudaEnabledDeviceCount
>>> print(cv2.cuda.getCudaEnabledDeviceCount())
1

```
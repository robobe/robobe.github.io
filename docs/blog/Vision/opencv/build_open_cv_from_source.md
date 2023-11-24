---
tags:
    - opencv
    - build
    - source
---

# Build opencv from source
Build opencv include python binding from source on ubuntu 22.04
build and install opencv python binding into python virtual env


## prerequisite

## source
clone from github

---

## configure

```bash title="show all build options"
cmake -LA
```

### ffmpeg
!!! 
    Set ffmpeg support to OFF
    There an error in build process
    for more information check [...](https://medium.com/@vladakuc/build-opencv-4-x-with-ffmpeg-from-source-on-ubuntu-22-04-c71df4c94aa7)
### gstreamer

```
-D WITH_GSTREAMER=ON
```

!!! tip gstreamer dependencies for opencv build
    ```bash
    sudo apt install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
    ```
     

### build into virtual environment

```bash title="create venv"
python3 -m venv venv
source venv/bin/activate

pip install numpy

```

```bash
cmake -LA | grep -i python3
#
BUILD_opencv_python3:BOOL=ON
PYTHON3_EXECUTABLE:FILEPATH=/usr/bin/python3
PYTHON3_INCLUDE_DIR:PATH=/usr/include/python3.10
PYTHON3_LIBRARY:FILEPATH=/usr/lib/x86_64-linux-gnu/libpython3.10.so
PYTHON3_NUMPY_INCLUDE_DIRS:PATH=/usr/lib/python3/dist-packages/numpy/core/include
PYTHON3_PACKAGES_PATH:STRING=lib/python3.10/dist-packages
```

#### one liners
```bash
# find executable path
python -c "import sys; print(sys.executable)"
# find lib path
python3 -c "import sysconfig; print(sysconfig.get_paths()['purelib'])"
# find include
python3 -c "import sysconfig; print(sysconfig.get_paths()['data'] + '/include')"
# numpy include
python3 -c "import numpy; print(numpy.get_include())"
# python install path
python3 -c "import sysconfig; print(sysconfig.get_paths()['purelib'])"
```

#### init cmake arguments
```bash
-D BUILD_opencv_python3=ON \
-D PYTHON3_EXECUTABLE=`python -c "import sys; print(sys.executable)"` \
-D PYTHON3_INCLUDE_DIR=`python3 -c "import sysconfig; print(sysconfig.get_paths()['data'] + '/include')"` \
-D PYTHON3_NUMPY_INCLUDE_DIRS=`python3 -c "import numpy; print(numpy.get_include())"` \
-D PYTHON3_PACKAGES_PATH=`python3 -c "import sysconfig; print(sysconfig.get_paths()['purelib'])"` \
-D OPENCV_PYTHON3_INSTALL_PATH=`python3 -c "import sysconfig; print(sysconfig.get_paths()['purelib'])"` \
```


---

### final 

```
cmake \
-D CMAKE_BUILD_TYPE=RELEASE  \
-D CMAKE_INSTALL_PREFIX=/usr/local \
-D WITH_FFMPEG=OFF \
-D BUILD_JAVA=OFF \
-D BUILD_opencv_python3=ON \
-D WITH_GSTREAMER=ON \
-D OPENCV_GENERATE_PKGCONFIG=ON \
-D INSTALL_PYTHON_EXAMPLES=ON \
-D INSTALL_C_EXAMPLES=OFF \
-D BUILD_EXAMPLES=ON \
-D PYTHON3_EXECUTABLE=`python -c "import sys; print(sys.executable)"` \
-D PYTHON3_INCLUDE_DIR=`python3 -c "import sysconfig; print(sysconfig.get_paths()['data'] + '/include')"` \
-D PYTHON3_NUMPY_INCLUDE_DIRS=`python3 -c "import numpy; print(numpy.get_include())"` \
-D PYTHON3_PACKAGES_PATH=`python3 -c "import sysconfig; print(sysconfig.get_paths()['purelib'])"` \
-D OPENCV_PYTHON3_INSTALL_PATH=`python3 -c "import sysconfig; print(sysconfig.get_paths()['purelib'])"` \

..

```

---

### examples
all examples install in `/usr/local/share/opencv4/samples/python`


---

## Reference
- [Compiling OpenCV good flags explain](https://www.simonwenkel.com/notes/software_libraries/opencv/compiling-opencv.html)
- [opencv with ffmpeg on ubuntu 22.04](https://medium.com/@vladakuc/build-opencv-4-x-with-ffmpeg-from-source-on-ubuntu-22-04-c71df4c94aa7)
- [Windows ](https://gist.github.com/amir-saniyan/9a6acb2815e42502ac899efc52b0f870)
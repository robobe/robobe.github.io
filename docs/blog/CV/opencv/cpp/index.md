---
title: OpenCV cpp tutorial
tags:
    - opencv
    - cpp
---

!!! note ""
    - `include <opencv2/opencv.hpp>`
    - Namespace `cv::`
    
     
```cpp main=""
#include <stdio.h>
#include <opencv2/opencv.hpp>

int main(int argc, char** argv )
{
    cv::Mat image;
    image = cv::imread("lana.png");
    if ( !image.data )
    {
        printf("No image data \n");
        return -1;
    }
    cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE );
    cv::imshow("Display Image", image);
    cv::waitKey(0);
    return 0;
}
```

```c title="CMakeLists.txt"
cmake_minimum_required(VERSION 3.22)
project(cpp_tutorial)

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
add_executable( cv_hello main.cpp )
target_link_libraries( cv_hello ${OpenCV_LIBS} )
```
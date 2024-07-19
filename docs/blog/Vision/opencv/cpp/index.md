---
tags:
    - opencv
    - cpp
---

     
## OpenCV VSCode project using cmake

Setup minimal project for build and development opencv depend project
using cmake and vscode


```cmake title="CMakeLists.txt"
cmake_minimum_required(VERSION 3.0)
project( cv_cpp_tutorial )

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable( hello hello.cpp )
target_link_libraries( hello ${OpenCV_LIBS} )
```


```cpp title="hello.cpp
#include <iostream>
#include <opencv2/opencv.hpp>

int main(){
    cv::Mat image(600, 800, CV_8UC3, cv::Scalar(100, 250, 30)); 
    cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE );
    cv::imshow("Display Image", image);
    cv::waitKey(0);
    return 0;
}
```

!!! note "include path"
    ```json
    "includePath": [
                "${workspaceFolder}/**",
                "/usr/include/opencv4/**"
            ],
    ```
     
```json title="c_cpp_properties.json"
{
    "configurations": [
        {
            "name": "Linux",
            "includePath": [
                "${workspaceFolder}/**",
                "/usr/include/opencv4/**"
            ],
            "defines": [],
            "compilerPath": "/usr/bin/gcc",
            "cStandard": "gnu17",
            "cppStandard": "gnu++14",
            "intelliSenseMode": "linux-gcc-x64"
        }
    ],
    "version": 4
}
```
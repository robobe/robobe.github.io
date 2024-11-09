---
tags:
    - ros2
    - camera
    - capture
    - image
    - usb
    - v4l
---

Capture USB camera on convert it to ROS2 image message

From simple google search i found this packages base on v4l or gstreamer

## Packages

| Package  | Desc  |
|---|---|
| [usb_cam](#usb_cam)  | This package is based off of V4L devices specifically instead of just UVC.  |
| gscam | ROS2 package for broadcasting any GStreamer video stream via image transport.|
| gscam2 | ROS2 package for broadcasting any GStreamer video stream via image transport. support intra process communication|
| [ros2_v4l2_camera](#v4l_camera) | A ROS 2 camera driver using Video4Linux2 (V4L2). use cv_bridge to build ros image message |
| opencv_cam | A simple ROS2 camera driver based on OpenCV. |




## usb_cam
This package is based off of V4L devices specifically instead of just UVC

[ROS wiki](http://wiki.ros.org/usb_cam#I.2FO_modes_reference)
[github](https://github.com/ros-drivers/usb_cam)
```
sudo apt install ros-humble-usb-cam
```



!!! tip device support formats
    When running the node it's output all available formats

    ```bash
    ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video4
    #
    Starting 'default_cam' (/dev/video4) at 640x480 via mmap (yuyv) at 30 FPS
    This device supports the following formats:
        YUYV 4:2:2 640 x 480 (30 Hz)
        YUYV 4:2:2 640 x 480 (24 Hz)
        YUYV 4:2:2 640 x 480 (20 Hz)
        YUYV 4:2:2 640 x 480 (15 Hz)
        YUYV 4:2:2 640 x 480 (10 Hz)
        YUYV 4:2:2 640 x 480 (7 Hz)
        YUYV 4:2:2 640 x 480 (5 Hz)
        ...
    ```
     
    from the above output we saw that the camera default is: `640x480 via mmap (yuyv) at 30 FPS`
    - width / height
    - io-mode: mmap (libusb memory mapping)
    - pixel-format: yuyv (YUV420)
    - fps: 30

### usage
```bash title=usage
ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video4
```

The node publish image msg accordingly to camera fps
```bash
ros2 topic hz /image_raw
average rate: 29.957
        min: 0.031s max: 0.036s std dev: 0.00185s window: 32
```

#### params

```bash title="fps"
ros2 run usb_cam usb_cam_node_exe \
    --ros-args -p video_device:=/dev/video4 -p framerate:=10.0
```

```bash title="width, height"
ros2 run usb_cam usb_cam_node_exe 
    --ros-args -p video_device:=/dev/video4 \
    -p framerate:=10.0 \
    -p image_width:=800 -p image_height:=600
```

##### camera_info_url

```
ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video4 \
-p  camera_info_url:=file:///home/user/tmp/camera.yaml
```

##### param file example
```yaml
/**:
    ros__parameters:
      video_device: "/dev/video2"
      framerate: 15.0
      io_method: "mmap"
      frame_id: "camera2"
      pixel_format: "mjpeg2rgb"
      av_device_format: "YUV422P"
      image_width: 640
      image_height: 480
      camera_name: "test_camera2"
      camera_info_url: "package://usb_cam/config/camera_info.yaml"
      brightness: -1
      contrast: -1
      saturation: -1
      sharpness: -1
      gain: -1
      auto_white_balance: true
      white_balance: 4000
      autoexposure: true
      exposure: 100
      autofocus: false
      focus: -1
```

#### Service
- **set_camera_info**: set camera info to be publish by `/camera_info`
- **set_capture** : Start/Stop capture


---

## gscam
[source](https://github.com/ros-drivers/gscam/tree/ros2)

```
sudo apt install ros-humble-usb-gscam
```

## gscam2
[source](https://github.com/clydemcqueen/gscam2)

Build from code

---

## v4l_camera
[source](https://gitlab.com/boldhearts/ros2_v4l2_camera)

A ROS 2 camera driver using Video4Linux2 (V4L2).

Features
- Lists and exposes all user-settable controls of your camera as ROS 2
- parameters.
- Uses cv_bridge to convert raw frames to ROS 2 messages, so
- supports a wide range of encoding conversions.
- Supports image_transport to enable compression.
- Supports composing the camera node and using ROS 2 intra-process commmunication with zero-copy messaging.

```
sudo apt install ros-humble-v4l-camera
```

```bash
ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:=/dev/video4
```

---

## opencv_cam

A simple ROS2 camera driver based on OpenCV.

---

## Test 

using i7 machine and Logitech C920 camera to test cpu usage for each of the packages
view image using rqt


|   | usb_cam  | v4l-camera  | gscam  | gscam2  |
|---|---|---|---|---|
| 640*480 10fps  | 0.7-1.3  |   |   |   |
| 640*480 30fps  | 2.7-3.3  |  11 |   |   |


---


## v4l utils
v4l-utils is a collection of utilities for handling video devices,

```
sudo apt install v4l-utils
```

### v4l2-ctl
The primary command-line tool to control video devices.
- Lists supported formats, resolutions, and other capabilities.
- Adjusts camera parameters like brightness, contrast, saturation, exposure, etc.
- Captures frames or short videos from the device.


#### list device

```bash
v4l2-ctl --list-devices
```

#### list supported formats
```bash 
v4l2-ctl -d /dev/video4 --list-formats
ioctl: VIDIOC_ENUM_FMT
        Type: Video Capture

        [0]: 'YUYV' (YUYV 4:2:2)
        [1]: 'H264' (H.264, compressed)
        [2]: 'MJPG' (Motion-JPEG, compressed)
```

```bash
v4l2-ctl -d /dev/video4 --list-formats-ext
#
[0]: 'YUYV' (YUYV 4:2:2)
                Size: Discrete 640x480
                        Interval: Discrete 0.033s (30.000 fps)
                        Interval: Discrete 0.042s (24.000 fps)
                        Interval: Discrete 0.050s (20.000 fps)
                ...
                        Interval: Discrete 0.067s (15.000 fps)
                        Interval: Discrete 0.100s (10.000 fps)
                        Interval: Discrete 0.133s (7.500 fps)
                        Interval: Discrete 0.200s (5.000 fps)
                Size: Discrete 1280x720
                        Interval: Discrete 0.100s (10.000 fps)
                        Interval: Discrete 0.133s (7.500 fps)
                        Interval: Discrete 0.200s (5.000 fps)
                Size: Discrete 1600x896
                        Interval: Discrete 0.133s (7.500 fps)
                        Interval: Discrete 0.200s (5.000 fps)
                Size: Discrete 1920x1080
                        Interval: Discrete 0.200s (5.000 fps)
                Size: Discrete 2304x1296
                        Interval: Discrete 0.500s (2.000 fps)
                Size: Discrete 2304x1536
                        Interval: Discrete 0.500s (2.000 fps)
```

---

### gstreamer pipes
gstreamer pipe example to get the request stream from the camera

- raw
- mjpeg
- h264

```bash
gst-launch-1.0 -vvv v4l2src device=/dev/video0 ! video/x-raw ! fakesink 
gst-launch-1.0 -vvv v4l2src device=/dev/video0 ! image/jpeg ! fakesink 

### raw
gst-launch-1.0 v4l2src device=/dev/video4 ! video/x-raw,width=800,height=600,framerate=10/1, format=YUY2 ! videoconvert ! fpsdisplaysink

gst-launch-1.0 v4l2src device=/dev/video4 ! video/x-raw,width=800,height=600,framerate=24/1, format=YUY2 ! videoconvert ! fpsdisplaysink

gst-launch-1.0 v4l2src device=/dev/video4 ! video/x-raw,width=864,height=480,framerate=15/1, format=YUY2 ! videoconvert ! fpsdisplaysink

gst-launch-1.0 v4l2src device=/dev/video4 ! video/x-raw,width=800,height=600,framerate=10/1, format=YUY2 ! videoconvert ! fpsdisplaysink

### mjpeg
gst-launch-1.0 v4l2src device=/dev/video4 ! image/jpeg,width=800,height=600,framerate=10/1 ! jpegdec ! videoconvert ! fpsdisplaysink

gst-launch-1.0 v4l2src device=/dev/video4 ! image/jpeg,width=800,height=600,framerate=10/1 ! jpegdec ! videoconvert ! fpsdisplaysink

### h264
gst-launch-1.0 uvch264src device=/dev/video4 name=src auto-start=true src.vidsrc ! video/x-h264,width=1280,height=720,framerate=30/1 ! h264parse ! vah264dec ! videoconvert ! fpsdisplaysink video-sink=fakesink
```

---

## Reference
- [BosonUSB](https://github.com/FLIR/BosonUSB)
- [Boson calibration](https://github.com/FLIR/flir_adk_ethernet/tree/master/example_calibrations)

---
tags:
    - ros2
    - opencv
    - cv_bridge
    - bridge
    - cpp
---

Pub / Sub Image using ROS2 CV_BRIDGE using **cyclonedds**

!!! note "check cyclonedds configuration"
    [ Reduce how eager CycloneDDS is in retransmits #484 ](https://github.com/eclipse-cyclonedds/cyclonedds/issues/484)

    ```xml title="cyclonedds.xml"
    <?xml version="1.0" encoding="UTF-8" ?>
    <CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
        <Domain Id="any">
            <Internal>
                <SocketReceiveBufferSize min="20MB"></SocketReceiveBufferSize>
            </Internal>
        </Domain>
    </CycloneDDS>
    ```

    ```bash title="config usage"
    export CYCLONEDDS_URI=file://$PWD/cyclonedds.xml
    ```

    ```bash title="os settings"
    echo 30000000 | sudo tee /proc/sys/net/core/rmem_max

    # or using /etc/sysctl.conf for persistence
    ```
     
---

## Demo
### Pub

```cpp title="simple_img_pub"
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/clock.hpp>
#include <opencv2/opencv.hpp>
#include <rmw/qos_profiles.h>
#include <rclcpp/qos.hpp>
#include <rclcpp/time.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/msg/header.hpp>

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
      : Node("minimal_publisher")
  {
    // auto qos = rclcpp::SensorDataQoS();
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
    // auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("topic", qos);
    timer_ = this->create_wall_timer(
        1000ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    cv::Mat img(cv::Size(640, 480), CV_8UC3);
    cv::randu(img, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255));
    rclcpp::Time now = this->get_clock()->now();
    // auto message = sensor_msgs::msg::Image();
    auto my_header = std_msgs::msg::Header();
    my_header.stamp = now;
    auto msg = cv_bridge::CvImage(my_header, "bgr8", img).toImageMsg();
    this->publisher_->publish(*msg.get());

    // RCLCPP_INFO_STREAM(this->get_logger(), ""<<(c-message.header.stamp).seconds());
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MinimalPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

### Sub

```cpp title="simple_img_sub"
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/time.hpp>
#include <rmw/qos_profiles.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class MinimalSub : public rclcpp::Node
{
public:
    MinimalSub() : Node("minimal_img_sub")
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "hello sub");
        // auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
        sub_ = this->create_subscription<sensor_msgs::msg::Image>("topic", qos, std::bind(&MinimalSub::img_handler, this, std::placeholders::_1));
    }

private:
    void img_handler(sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg);

        auto current = this->get_clock()->now();
        RCLCPP_INFO_STREAM(this->get_logger(), "" << (current - msg->header.stamp).seconds());

        cv::imshow("debug", cv_ptr->image);
        cv::waitKey(1);
    }
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MinimalSub>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

### cmake

```cmake
cmake_minimum_required(VERSION 3.8)
project(cpp_demos)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include /opt/ros/humble/include/cv_bridge ${OpenCV_INCLUDE_DIRS})

add_executable(simple_img_pub src/simple_img_pub.cpp)
ament_target_dependencies(simple_img_pub rclcpp sensor_msgs cv_bridge)

add_executable(simple_img_sub src/simple_img_sub.cpp)
ament_target_dependencies(simple_img_sub rclcpp sensor_msgs cv_bridge OpenCV)

install(TARGETS
  simple_img_pub
  simple_img_sub
  DESTINATION lib/${PROJECT_NAME})

ament_package()

```

### Usage

!!! note "cyclonedds configuration usage"
     ```
     export CYCLONEDDS_URI=file://$PWD/cyclonedds.xml
     ```

```bash title="publisher"
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2 run cpp_demos simple_img_pub
```

```bash title"subscriber"
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp  ros2 run cpp_demos simple_img_sub
```

!!! note transport cost
    delta time between pub and sub include cv_bridge parsing in sec
    ```
    [INFO] [1707023401.506130604] [minimal_img_sub]: 0.00126415
    [INFO] [1707023402.506277985] [minimal_img_sub]: 0.00141274
    [INFO] [1707023403.506211791] [minimal_img_sub]: 0.00119082
    [INFO] [1707023404.506217749] [minimal_img_sub]: 0.00128191
    [INFO] [1707023405.506714684] [minimal_img_sub]: 0.00156843
    ```
     
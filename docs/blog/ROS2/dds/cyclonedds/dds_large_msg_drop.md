---
tags:
    - ros
    - dds
    - cyclonedds
    - image
    - large message
---
# Send image message across ROS using cyclonedds

- Ubuntu 22.04
- Humble
- Cyclonedds


## Demo
publish Image 

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

Increased my `/proc/sys/net/core/rmem_max` to 30MB

```bash
echo 30000000 | sudo tee /proc/sys/net/core/rmem_max
```

### use cyclonedds.xml  

```bash
export CYCLONEDDS_URI=file://$PWD/cyclonedds.xml
```

```bash title="publisher"
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2 run cpp_demos simple_img_pub
```

```bash title="subscriber"
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp  ros2 run cpp_demos simple_img_sub 
```

---

```cpp title="simple_img_pub.cpp"
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
// #include <std_msgs/msg/string.hpp>
// #include <sensor_msgs/msg/image.hpp>
// #include <cv_bridge/cv_bridge.h>
// #include <std_msgs/msg/header.hpp>
#include "custom_msg/msg/image.hpp"

using namespace std::chrono_literals;

void fill_loaned_message(
  rclcpp::LoanedMessage<custom_msg::msg::Image> & loanedMsg,
  const cv::Mat & frame, uint64_t timestamp,
  uint64_t count)
{
  auto &msg = loanedMsg.get();
  auto size = frame.elemSize() * frame.total();
  if (size > custom_msg::msg::Image::MAX_SIZE) {
    std::stringstream s;
    s << "MAX_SIZE exceeded - message requires " << size << "bytes\n";
    throw std::runtime_error(s.str());
  }

  msg.rows = frame.rows;
  msg.cols = frame.cols;
  msg.size = size;
  msg.channels = frame.channels();
  msg.type = frame.type();
  msg.offset = 0;  // TODO(matthiaskillat) alignment?
  msg.count = count;
  msg.timestamp = timestamp;

  // TODO(matthiaskillat): avoid if possible
  std::memcpy(msg.data.data(), frame.data, size);
}

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
      : Node("minimal_publisher")
  {
    // auto qos = rclcpp::SensorDataQoS();
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
    // auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
    // publisher_ = this->create_publisher<sensor_msgs::msg::Image>("topic", qos);
    publisher_ = this->create_publisher<custom_msg::msg::Image>("topic", qos);
    timer_ = this->create_wall_timer(
        1000ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    cv::Mat img(cv::Size(640, 480), CV_8UC3);
    auto size = img.elemSize()*img.total();
    RCLCPP_INFO_STREAM(this->get_logger(), size);
    cv::randu(img, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255));
    rclcpp::Time now = this->get_clock()->now();
    // auto message = sensor_msgs::msg::Image();
    // auto my_header = std_msgs::msg::Header();
    // my_header.stamp = now;
    auto sec_as_tin64 = static_cast<int64>(now.seconds() * 1e9);
    RCLCPP_INFO_STREAM(this->get_logger(), sec_as_tin64);
    auto msg = this->publisher_->borrow_loaned_message();
    fill_loaned_message(msg,
      img,
      sec_as_tin64,
      fid_++);
    // auto msg = cv_bridge::CvImage(my_header, "bgr8", img).toImageMsg();
    this->publisher_->publish(std::move(msg));

    // RCLCPP_INFO_STREAM(this->get_logger(), ""<<(c-message.header.stamp).seconds());
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<custom_msg::msg::Image>::SharedPtr publisher_;
  int fid_ = 0;
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

```cpp title="simple_img_sub.cpp"
#include <memory>
#include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/image.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/time.hpp>
#include <rmw/qos_profiles.h>
// #include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "custom_msg/msg/image.hpp"

void from_message(const custom_msg::msg::Image::SharedPtr &msg, cv::Mat &frame)
{
    void *buffer = msg->data.data();
    frame = cv::Mat(msg->rows, msg->cols, msg->type, buffer);
}

class MinimalSub : public rclcpp::Node
{
public:
    MinimalSub() : Node("minimal_img_sub")
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "hello sub");
        // auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
        sub_ = this->create_subscription<custom_msg::msg::Image>("topic", qos, std::bind(&MinimalSub::img_handler, this, std::placeholders::_1));
    }

private:
    void img_handler(custom_msg::msg::Image::SharedPtr msg)
    {
        cv::Mat frame;
        from_message(msg, frame);

        auto now = this->get_clock()->now();
        auto sec_as_tin64 = static_cast<int64>(now.seconds() * 1e9);

        // auto msg_time = rclcpp::Duration::from_seconds(msg->timestamp);
        RCLCPP_INFO_STREAM(this->get_logger(), (sec_as_tin64 - msg->timestamp)/1e9);
        cv::imshow("debug", frame);
        cv::waitKey(1);
    }
    rclcpp::Subscription<custom_msg::msg::Image>::SharedPtr sub_;
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

```bash title="Image.msg"
uint32 rows
uint32 cols
uint8 channels
uint8 type
uint32 offset
uint32 size
uint64 count
uint64 timestamp

uint32 MAX_SIZE=921600
char[921600] data
```

```bash title="CMakeLists.txt"
cmake_minimum_required(VERSION 3.8)
project(cpp_demos)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(custom_msg REQUIRED)
find_package(example_interfaces REQUIRED)

include_directories(include ${OpenCV_INCLUDE_DIRS})



# # img pub
add_executable(simple_img_pub src/simple_img_pub.cpp)
ament_target_dependencies(simple_img_pub rclcpp custom_msg OpenCV)

# # img sub
add_executable(simple_img_sub src/simple_img_sub.cpp)
ament_target_dependencies(simple_img_sub rclcpp  OpenCV custom_msg)

install(TARGETS
  simple_img_pub
  simple_img_sub
  DESTINATION lib/${PROJECT_NAME})


ament_package()



```
## Reference
- [ Reduce how eager CycloneDDS is in retransmits #484 ](https://github.com/eclipse-cyclonedds/cyclonedds/issues/484)
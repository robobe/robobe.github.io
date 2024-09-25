---
tags:
    - ros2
    - diagnostic
    - aggregator
    - plugin
    - analyzer
---


```bash
CustomDiagnostics
├── CMakeLists.txt
├── include
│   └── demo_analyzer.hpp
├── package.xml
├── plugins.xml
└── src
    └── demo_analyzer.cpp
```

### Header

This defines the header for our diagnostic analyzer. We're defining all the pure virtual functions from the diagnostic_aggregator::Analyzer base class, i.e.
- init
- match: match status item name, return true to handle this item
- analyze: In the analyze() function, we return true if we will report that item in our output.
- report: Report is called at 1Hz intervals. Analyzers should return a vector of processed DiagnosticStatus messages. 
- getPath
- getName

```cpp title="include/demo_analyzer.hpp"
#ifndef DIAGNOSTIC_AGGREGATOR__DEMO_ANALYZER_HPP_
#define DIAGNOSTIC_AGGREGATOR__DEMO_ANALYZER_HPP_

#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_aggregator/analyzer.hpp>
#include <diagnostic_aggregator/visibility_control.hpp>

namespace diagnostic_aggregator
{
    class DemoAnalyzer : public Analyzer
    {
    public:
        DIAGNOSTIC_AGGREGATOR_PUBLIC
        DemoAnalyzer();

        DIAGNOSTIC_AGGREGATOR_PUBLIC
        ~DemoAnalyzer();


        DIAGNOSTIC_AGGREGATOR_PUBLIC
        bool init(
            const std::string &base_path, const std::string &breadcrumb,
            const rclcpp::Node::SharedPtr node);

        bool match(const std::string &name)
        {
            (void)name;

            return false;
        }

        bool analyze(std::shared_ptr<StatusItem> item)
        {
            (void)item;

            return false;
        }

        DIAGNOSTIC_AGGREGATOR_PUBLIC
        virtual std::vector<std::shared_ptr<diagnostic_msgs::msg::DiagnosticStatus>> report();

        std::string getPath() const { return ""; }
        std::string getName() const { return ""; }
    };

} // namespace diagnostic_aggregator

#endif // DIAGNOSTIC_AGGREGATOR__DEMO_ANALYZER_HPP_
```

```cpp title="src/demo_analyzer.cpp"
#include "demo_analyzer.hpp"
#include <memory>
#include <string>
#include <vector>

namespace diagnostic_aggregator
{

    DemoAnalyzer::DemoAnalyzer() {}
    DemoAnalyzer::~DemoAnalyzer() {}
    std::vector<std::shared_ptr<diagnostic_msgs::msg::DiagnosticStatus>> report()
    {
        std::vector<std::shared_ptr<diagnostic_msgs::msg::DiagnosticStatus>> processed;

        return processed;
    }

    bool DemoAnalyzer::init(
        const std::string &base_path, const std::string &breadcrumb, const rclcpp::Node::SharedPtr node)
    {
        (void)base_path;
        (void)breadcrumb;
        (void)node;

        RCLCPP_INFO(rclcpp::get_logger("DemoAnalyzer"), "---demo analyzer ----");

        return true;
    }

    std::vector<std::shared_ptr<diagnostic_msgs::msg::DiagnosticStatus>> DemoAnalyzer::report()
    {
        std::vector<std::shared_ptr<diagnostic_msgs::msg::DiagnosticStatus>> processed;

        return processed;
    }

}

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(diagnostic_aggregator::DemoAnalyzer, diagnostic_aggregator::Analyzer)

```

---

### CMakeLists

```c title="CMakeLists.txt"
cmake_minimum_required(VERSION 3.8)
project(CustomDiagnostics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_aggregator REQUIRED)

pluginlib_export_plugin_description_file(CustomDiagnostics plugins.xml)


set(dependencies
  rclcpp
  diagnostic_msgs
  diagnostic_aggregator
  pluginlib
)


add_library(CustomDiagnostics SHARED
  src/demo_analyzer.cpp
)
target_compile_features(CustomDiagnostics PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_include_directories(CustomDiagnostics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(CustomDiagnostics
  ${dependencies}
)

target_compile_definitions(CustomDiagnostics PRIVATE "POLYGON_PLUGINS_BUILDING_LIBRARY")
target_compile_definitions(CustomDiagnostics PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")


install(
  TARGETS CustomDiagnostics
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_libraries(
  polygon_plugins
)
ament_export_targets(
  export_${PROJECT_NAME}
)
ament_package()

```

#### pluginlib_export_plugin_description_file

```
pluginlib_export_plugin_description_file(<base_package> <xml_file>)
```


<base_package> is the name of the **package** containing the base class for the plugins. (base class ?? for me its work only with the package that contain the header)
<xml_file> is the relative path to the XML file describing the plugins

---

### XML plugin description

```xml title="plugins.xml"
<library path="CustomDiagnostics">
    <class name="DemoAnalyzer"
        type="diagnostic_aggregator::DemoAnalyzer"
        base_class_type="diagnostic_aggregator::Analyzer">
        <description>
            Demo
        </description>
    </class>
</library>
```

---

## Usage

```yaml
analyzers:
  ros__parameters:
    path: Aggregation
     new:
       type: DemoAnalyzer
       path: hb_data_new
       startswith: [ '/hb' ]
```

```
ros2 run diagnostic_aggregator aggregator_node  --ros-args --params-file
```
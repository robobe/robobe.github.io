---
title: linters
tags:
    - ros2
    - linter
---

Lint, or a linter, is a static code analysis tool used to flag programming errors, bugs, stylistic errors and suspicious constructs

---

# ament_cmake_clang_format

Checks the code style of C / C++ source files using ClangFormat

## install 

```bash
sudo apt install ros-humble-ament-cmake-clang-format
```

## usage
- CMakeLists settings
- package.xml
- cli commands


```cpp title="CMakeLists.txt"
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()
```

`ament_lint_auto_find_test_dependencies` function search for linter's in `package.xml`

```xml title="package.xml"
<test_depend>ament_lint_auto</test_depend>
<test_depend>ament_cmake_clang_format</test_depend>
```

```bash
# build
colcon build --packages-select cpp_tutrial_pkg 
# run test / linters
colcon test --packages-select cpp_tutrial_pkg 
# show test results
colcon test-result
colcon test-result --verbose
# run the linter directly
ament_clang_format src/tutorials/cpp_tutrial_pkg/
# fix the problem's
ament_clang_format src/tutorials/cpp_tutrial_pkg/ --reformat
# run colcon test again, no errors
colcon test --packages-select cpp_tutrial_pkg 
# show test results, no errors
colcon test-result
```

!!! tip "clear test result"
    [test_result](https://colcon.readthedocs.io/en/released/reference/verb/test-result.html)

    ```
    colcon test-result --delete
    ```

     
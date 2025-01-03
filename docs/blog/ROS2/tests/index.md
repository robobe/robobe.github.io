---
tags:
    - ros2
    - pytest
    - tests
    - unit tests
    - cmake
---

# ROS2 unit test with PyTest
How to add tests to python package base on `ament_cmake`

```python title="test/test_simple.py"

import pytest

def test_simple():
    assert 1 == 1

```

```bash title="CMakeLists.txt"
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
 
  ament_add_pytest_test(minimal_publisher_test test/test_simple.py
   TIMEOUT 60
  )
endif()
```

```xml title="package.xml"
<test_depend>ament_cmake_pytest</test_depend>
```

## build and test

```bash
colcon test
# control the output display
colcon --event-handlers console_direct+
```

`event-handlers` control how colcon display events during build process

`console_direct+`: `+` enable print output from each package as it is produced

### check last test running
Display the results of tests that have been executed using the `colcon test` command.

```bash
colcon test-result --all
colcon test-result --all --verbose
```

`--verbose`: Display detailed information, test case that failed and their error message

`--all`: Display test result for all packages

---

## Test Pub/Sub

```bash
pip install pytest-asyncio
```

In this demo the test function publish to it's self

```python
import asyncio
import pytest
import rclpy
from std_msgs.msg import String

@pytest.fixture
def ros2_setup():
    """Fixture to initialize and shutdown ROS 2."""
    rclpy.init()
    yield
    rclpy.shutdown()

@pytest.mark.asyncio
async def test_node_response(ros2_setup):
    """Test a node's ability to respond to an input message."""
    # Create a mock ROS 2 node
    test_node = rclpy.create_node("test_node")
    logger = test_node.get_logger()
    # Prepare results storage
    received_messages = []

  
    # Define a callback to capture messages on `/output`
    def output_callback(msg):
        received_messages.append(msg.data)

    publisher = test_node.create_publisher(String, "/input", 10)
    # Create a subscription to the `/output` topic
    test_node.create_subscription(String, "/input", output_callback, 10)

    

    # Wait for the entities to establish connections
    await asyncio.sleep(2)

    # Publish a message to the `/input` topic
    input_message = String()
    input_message.data = "Hello, node!"
    publisher.publish(input_message)



    for i in range(10):  # Check for messages over 1 second
        logger.info(f"iteration {i}")
        rclpy.spin_once(test_node, timeout_sec=0.1)
        if received_messages:
            break

    # Check if the node under test published a response
    assert len(received_messages) > 0, "No messages received on /output"

    # Cleanup
    test_node.destroy_node()
```
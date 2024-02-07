---
tags:
    - ros2
    - state machine
    - yasmin
---

# YASMIN (Yet Another State MachINe)

YASMIN is a project focused on implementing robot behaviors using Finite State Machines (FSM). It is available for ROS 2, Python and C++.


## Demo

- Declare Two state and final outcome
- Declare Blackboard to pass data between state
- Add state machine viewer

```python title="simple_demo"
import time
import rclpy
from rclpy.node import Node
from rclpy import logging

from yasmin import State
from yasmin import Blackboard
from yasmin import StateMachine
from yasmin_viewer import YasminViewerPub

log = logging.get_logger(__name__)

# define state Foo
class FooState(State):
    def __init__(self) -> None:
        super().__init__(["outcome1", "outcome2"])
        self.counter = 0

    def execute(self, blackboard: Blackboard) -> str:
        log.info("Executing state FOO")
        time.sleep(1)

        if self.counter < 10:
            self.counter += 1
            blackboard.foo_str = f"Counter: {self.counter}"
            return "outcome1"
        else:
            return "outcome2"


# define state Bar
class BarState(State):
    def __init__(self) -> None:
        super().__init__(outcomes=["outcome3"])

    def execute(self, blackboard: Blackboard) -> str:
        log.info("Executing state BAR")
        log.info(f"foo_str update by Foo state: {blackboard.foo_str}")
        time.sleep(1)

        return "outcome3"


class DemoNode(Node):

    def __init__(self) -> None:
        super().__init__("yasmin_node")
        log.info("start state machine v: 0.1")
        # create a state machine
        sm = StateMachine(outcomes=["outcome4"])

        # add states
        sm.add_state("FOO", FooState(),
                     transitions={"outcome1": "BAR",
                                  "outcome2": "outcome4"})
        sm.add_state("BAR", BarState(),
                     transitions={"outcome3": "FOO"})

        # pub
        YasminViewerPub(self, "YASMIN_DEMO", sm)

        # execute
        outcome = sm()
        log.info(outcome)


# main
def main(args=None):
    log.info("yasmin_demo")
    rclpy.init(args=args)
    node = DemoNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

```


## YASMIN Viewer
This viewer allows monitoring YASMIN's FSM. It is implemented with Flask and ReactJS. A filter is provided to show only one FSM.
[YASMIN github](https://github.com/uleroboticsgroup/yasmin#yasmin-viewer)


---

## devcontainer 
- Mapping port 5000 for `YASMIN Viewer`
- Set dockerfile context for requirements.txt
- Docker files
  - install missing packages
  - install python packages


```Dockerfile title="Dockerfile"
FROM althack/ros2:humble-dev 

# ** [Optional] Uncomment this section to install additional packages. **
#
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
   && apt-get -y install --no-install-recommends  \
    ros-humble-example-interface \
    ros-humble-action-tutorials-interfaces \
   # Clean up
   && apt-get autoremove -y \
   && apt-get clean -y \
   && rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=dialog

COPY src/yasmin/requirements.txt /tmp/requirements.txt
RUN pip3 install -r /tmp/requirements.txt
# Set up auto-source of workspace for ros user
ARG WORKSPACE
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc
```


```json title="".devcontainer.json
{
    "name": "YASMIN",
    "dockerFile": "Dockerfile",
    "workspaceFolder": "/workspaces/yasmin_ws",
    "remoteUser": "ros",
    "context": "..",
    "build": {
        
    },
    "runArgs": [
        "--hostname=dev",
        "--volume=/tmp/.X11-unix:/tmp/.X11-unix"
    ],
    "containerEnv": {
		"DISPLAY": "${localEnv:DISPLAY}" // Needed for GUI try ":0" for windows
    },
    "appPort": [5000],
    "customizations": {
        "vscode": {
 
    }
}
```
---

## Reference
- [yasmin github](https://github.com/uleroboticsgroup/yasmin)


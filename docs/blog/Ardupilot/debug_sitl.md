---
tags:
    - ardupilot
    - SITL
    - debug
---

# Debug Ardupilot
[Ardupilot Debugging with GDB using VSCode](https://ardupilot.org/dev/docs/debugging-with-gdb-using-vscode.html)

## Setting up SITL
### Config and Build
```
./waf configure --debug
```

#### build with debug symbol

```bash
# ardupilot/Tools/autotest
./sim_vehicle.py -v ArduCopter -f quad -D

# -D, --debug         build with debugging
```

!!! tip "check if has debug symbols"
    ```bash linenums="1" hl_lines="4"
    file arducopter
    #
    arducopter: ELF 64-bit LSB pie executable, x86-64, version 1 (GNU/Linux), dynamically linked, interpreter /lib64/ld-linux-x86-64.so.2, BuildID[sha1]=eb51dfafe7a6f361a1f661d012e9cdd8006b4595, for GNU/Linux 3.2.0, 
    with debug_info,
    not stripped
    ```

## Add VSCode launch config

```json title="attach"
{
    "name": "(gdb) Attach Copter",
    "type": "cppdbg",
    "request": "attach",
    "program": "${workspaceFolder}/build/sitl/bin/arducopter",
    "processId": "${command:pickProcess}",
    "MIMode": "gdb",
    "setupCommands": [
        {
            "description": "Enable pretty-printing for gdb",
            "text": "-enable-pretty-printing",
            "ignoreFailures": true
        }
    ]
}
```

## Set breakpoint
### Demo handleMessage
Entry points from mavlink message from GCS or CC

Ardupilot code stuct with common function for all type of vehicles (copter, plan, rover ..) and specific implementation for vehicle (ArduCopter)

- ArduCopter/GCS_Mavlink.cpp
- libraries/GCS_MAVLink/GCS_Common

To catch entry point for message handler place breakpoint at:

- Set breakpoint at `GCS_MAVLINK::handle_common_message` file: GCS_Common.cpp
- Set breakpoint at `GCS_MAVLINK_Copter::handleMessage` file: ArduCopter/GCS_Mavlink.cpp
- The above function has case conditions the default one call the `GCS_MAVLINK::handle_common_message` function: `handle_common_message`
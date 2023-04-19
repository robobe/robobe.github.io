---
tags:
    - ardupilot
    - system_time
---

# SYSTEM_TIME
[mavlink.io](https://mavlink.io/en/messages/common.html#SYSTEM_TIME)

Ardupilot send and received this message

## Sending
Implement by `GCS_Common` `send_system_time` method

the unix time calc from RTC class and return only if we have time source
the rtc_source_type is set by logic in `AP_RTC::set_utc_usec` that call by
- GPS when we have 3D fix lock or better
- Mavlink message `SYSTEM_TIME`
- ?

```cpp title="ardupilot time source enum"
// ordering is important in source_type; lower-numbered is
// considered a better time source.  These values are documented
// and used in the parameters!
enum source_type : uint8_t {
    SOURCE_GPS = 0,
    SOURCE_MAVLINK_SYSTEM_TIME = 1,
    SOURCE_HW = 2,
    SOURCE_NONE,
}
```

---


## Demo
Listen to `SYSTEM_TIME` message with and without GPS source

```ini
BRD_RTC_TYPES 0
GPS_TYPE 0
```

```bash title="without gps "
2023-04-16 19:23:29,160 - __main__ - INFO - param_name: BRD_RTC_TYPES value: 1.0
2023-04-16 19:23:30,151 - __main__ - INFO - {'mavpackettype': 'SYSTEM_TIME', 'time_unix_usec': 0, 'time_boot_ms': 2154028}
2023-04-16 19:23:31,152 - __main__ - INFO - {'mavpackettype': 'SYSTEM_TIME', 'time_unix_usec': 0, 'time_boot_ms': 2155028}
2023-04-16 19:23:32,155 - __main__ - INFO - {'mavpackettype': 'SYSTEM_TIME', 'time_unix_usec': 0, 'time_boot_ms': 2156028}
2023-04-16 19:23:33,154 - __main__ - INFO - {'mavpackettype': 'SYSTEM_TIME', 'time_unix_usec': 0, 'time_boot_ms': 2157028}
2023-04-16 19:23:34,152 - __main__ - INFO - {'mavpackettype': 'SYSTEM_TIME', 'time_unix_usec': 0, 'time_boot_ms': 2158028}
```

```ini
BRD_RTC_TYPES 1
GPS_TYPE 1
```

```bash title="with gps"
2023-04-16 19:27:05,833 - __main__ - INFO - Set SYSRM_TIME message interval to 1sec
2023-04-16 19:27:05,840 - __main__ - INFO - param_name: BRD_RTC_TYPES value: 1.0
2023-04-16 19:27:06,836 - __main__ - INFO - {'mavpackettype': 'SYSTEM_TIME', 'time_unix_usec': 0, 'time_boot_ms': 4028}
2023-04-16 19:27:07,841 - __main__ - INFO - {'mavpackettype': 'SYSTEM_TIME', 'time_unix_usec': 0, 'time_boot_ms': 5028}
2023-04-16 19:27:08,837 - __main__ - INFO - {'mavpackettype': 'SYSTEM_TIME', 'time_unix_usec': 0, 'time_boot_ms': 6028}
2023-04-16 19:27:09,836 - __main__ - INFO - {'mavpackettype': 'SYSTEM_TIME', 'time_unix_usec': 0, 'time_boot_ms': 7028}
2023-04-16 19:27:10,837 - __main__ - INFO - {'mavpackettype': 'SYSTEM_TIME', 'time_unix_usec': 0, 'time_boot_ms': 8028}
2023-04-16 19:27:11,842 - __main__ - INFO - {'mavpackettype': 'SYSTEM_TIME', 'time_unix_usec': 1681662430861592, 'time_boot_ms': 9028}
2023-04-16 19:27:12,837 - __main__ - INFO - {'mavpackettype': 'SYSTEM_TIME', 'time_unix_usec': 1681662431861192, 'time_boot_ms': 10028}
2023-04-16 19:27:13,840 - __main__ - INFO - {'mavpackettype': 'SYSTEM_TIME', 'time_unix_usec': 1681662432861625, 'time_boot_ms': 11028}
```

!!! note "time_unix"
     No epoc time until GPS has 3D fix

## Demo II
Send `SYSTEM_TIME` message to send ardupilot with time source when no GPS found

```ini
BRD_RTC_TYPES 2
GPS_TYPE 0
```

!!! note "BRD_RTC_TYPES"
    `BRD_RTC_TYPES` must be SOURCE_HW and not SOURCE_MAVLINK_SYSTEM_TIME to pass the condition
    ```cpp
    if (!(allowed_types & (1<<type))) {
        return;
    }
    ```
    from  AP_RTC.cpp file


```bash
2023-04-16 22:07:18,843 - __main__ - INFO - {'mavpackettype': 'SYSTEM_TIME', 'time_unix_usec': 1681671783248796, 'time_boot_ms': 573026}
2023-04-16 22:07:18,844 - __main__ - INFO - param_name: BRD_RTC_TYPES value: 2.0
2023-04-16 22:07:19,839 - __main__ - INFO - {'mavpackettype': 'SYSTEM_TIME', 'time_unix_usec': 1681671784248396, 'time_boot_ms': 574026}
2023-04-16 22:07:20,840 - __main__ - INFO - {'mavpackettype': 'SYSTEM_TIME', 'time_unix_usec': 1681671785248829, 'time_boot_ms': 575026}
2023-04-16 22:07:21,838 - __main__ - INFO - {'mavpackettype': 'SYSTEM_TIME', 'time_unix_usec': 1681671786248429, 'time_boot_ms': 576026}
```

!!! warning "BRD_RTC_TYPES"
    BRD_RTC_TYPES must be `SOURCE_HW` for ardupilot accept mavlink message "SYSTEM_TIME"


```python title="check code"
import time
import logging
import os
os.environ["MAVLINK20"]="1"
from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega

FMT = "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
logging.basicConfig(format=FMT, level=logging.INFO)
log = logging.getLogger(__name__)

# Create the connection
# master = mavutil.mavlink_connection("/dev/ttyACM0")
# master = mavutil.mavlink_connection("tcp:0.0.0.0:5760")
master = mavutil.mavlink_connection("udp:127.0.0.1:14550")
ONE_SEC = 1e6

def set_message_interval(interval_us):
    master.mav.command_long_send(
    master.target_system,
    master.target_component,
    ardupilotmega.MAV_CMD_SET_MESSAGE_INTERVAL,
    0,
    ardupilotmega.MAVLINK_MSG_ID_SYSTEM_TIME,
    interval_us,
    0,
    0,
    0,
    0,
    0
    )

def set_system_time():
    # while True:
    current = int(time.time()*1e6)
    log.info("%s" , current)
    master.mav.system_time_send(
    current,
    0)
    time.sleep(1/1)

# Wait a heartbeat before sending commands
master.wait_heartbeat()

set_message_interval(ONE_SEC)
set_system_time()

USE_PARAM_ID = -1
master.mav.param_request_read_send(
    master.target_system,
    master.target_component,
    b"BRD_RTC_TYPES",
    USE_PARAM_ID
)

while True:
    msg = master.recv_match()
    if not msg:
        continue
    if msg.get_type() == "SYSTEM_TIME":
        log.info("%s", msg.to_dict())
    if msg.get_type() == "PARAM_VALUE":
        message = msg.to_dict()
        param_name = message["param_id"]
        if param_name == "BRD_RTC_TYPES":
            param_value = message["param_value"]
            log.info("param_name: %s value: %s", param_name, param_value)

```
     
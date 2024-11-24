---
tags:
    - mavlink
    - pymavlink
---

# Pymavlink

## Build from source
- Clone [pymavlink](https://github.com/ArduPilot/pymavlink/tree/master)
- Clone [mavlink](https://github.com/ArduPilot/mavlink.git)


```bash
sudo apt-get install libxml2-dev libxslt-dev
# create venv
python -m venv venv
source venv/bin/activate
python -m pip install --upgrade future lxml pyserial
export MAVLINK_DIALECT_PATH=~/git/mavlink/message_definitions/v1.0/
pip install -e .
```


---

## pymavlink utils

### mavftp

```bash
#usb
python mavftp.py --device /dev/ttyACM0 getparams /tmp/params

# telem
code ```


```
mavproxy.py --master /dev/ttyUSB0 --baudrate 500000
ftp get @PARAM/param.pck
```


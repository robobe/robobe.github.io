---
tags:
    - arduino
    - mega
    - i2c
    - 
---

# Use Arduino as I2C slave

Config arduino as i2c slave, read data using cli and python

!!! note 
     using CP2112 as usb i2c bridge to connect pc to i2c bus


## pre

```
sudo apt install i2c-tools
```

### Permission
Set permission using one of the method:
- Add user to `i2c` group 
- Change permission using udev rule


Add user to `i2c` group
```bash
sudo usermod -aG i2c $USER
```

### Check i2c bus

```bash
i2cdetect -l
#
i2c-0   unknown         SMBus I801 adapter at efa0              N/A
i2c-1   unknown         Synopsys DesignWare I2C adapter         N/A
...
i2c-16  unknown         CP2112 SMBus Bridge on hidraw3          N/A
```

find i2c bridge on port 16

#### find slave address
```bash
i2cdetect -y -r 16
#
[sudo] password for user: 
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:                         -- -- -- -- -- -- -- -- 
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
30: -- -- -- -- -- -- -- -- -- -- 3a -- -- -- -- -- 
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
```

---

## Arduino

```cpp
#include <Arduino.h>
#include <Wire.h>

void requestEvent();
void receiveEvent(int howMany);

char reg = 0;
void setup() {
    Wire.begin(0x3a);
    Wire.onRequest(requestEvent);
    Wire.onReceive(receiveEvent);
    Serial.begin(9600);
    Serial.println("--start--");
}

void loop() {
    delay(100);
    
}

void receiveEvent(int howMany){
  Serial.println("receive");
  reg = Wire.read();
  Serial.println(reg, HEX);
}
void requestEvent() {
  int data = 0;
  Serial.println("request");
  Serial.println(reg, HEX);
  if (reg == 0x01) {
    data = 1024;  
  }
  else if (reg == 0x02) {
    data = 2048;  
  }
  else if (reg == 0x03) {
    data = 3072;  
  }
  else if (reg == 0x04) {
    data = 4096;  
  }
  Wire.write((byte*)&data, sizeof(data)); // Send the data to the master
}
```

## Cli usage
### Get

```bash
i2cget -y 16 0x3a 0x1 w
0x0400

i2cget -y 16 0x3a 0x2 w
0x0800
```

### Set

```bash
i2cset -y 16 0x3a 0x01
```

--- 

## Python

```
pip install smbus2
```

### Set
```python title="set data"
import smbus2

bus = smbus2.SMBus(16)
bus.write_byte(0x3a, 0x010)
```

### Get

```python
import smbus2

bus = smbus2.SMBus(16)
data = bus.read_block_data(0x3a, 0x01, 2)
data = bus.read_i2c_block_data(0x3a, 0x02, 2)
# data =bus.read_byte_data(0x3a, 0x01)
print(data)
# convert bytes to int
print(int.from_bytes(data, "little"))
```

!!! warning "read_byte_data"
    method `read_byte_data` return empty list
     

---

### platformio
Config for arduino mega

```ini
[env:megaatmega2560]
platform = atmelavr
board = megaatmega2560
framework = arduino
upload_port = /dev/ttyACM0
monitor_port = /dev/ttyACM0
```
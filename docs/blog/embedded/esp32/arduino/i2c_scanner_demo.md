---
tags:
    - esp32
    - arduino
    - platformio
    - i2c
    - mp6050
---

# i2c scanner

Scan for i2c devices, for demonstration connect mpu_6050

!!! tip "include wire library"
    ```ini
    [env:denky32]
    platform = espressif32
    framework = arduino
    board = denky32
    monitor_speed = 115200
    upload_port = /dev/ttyUSB0

    lib_deps =
        Wire
    ```
     

```cpp title="scan.cpp"
#include <Arduino.h>
#include <Wire.h>
 
void setup() {
  Wire.begin();
  Serial.begin(115200);
  Serial.println("\nI2C Scanner");
}
 
void loop() {
  byte error, address;
  int nDevices;
  Serial.println("Scanning...");
  nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
      nDevices++;
    }
    else if (error==4) {
      Serial.print("Unknow error at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  }
  else {
    Serial.println("done\n");
  }
  delay(5000);          
}
```

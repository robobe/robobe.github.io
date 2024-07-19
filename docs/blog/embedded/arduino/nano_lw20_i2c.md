---
tags:
    - arduino
    - i2c
    - nano
    - vscode
    - platformio
    - sensors
    - lw20
    - lidar
---

## Wiring

![alt text](images/arduino_nano_pinout.png)

![alt text](images/lw20_i2c_pinout.png)


| Arduino nano  | lw20 i2c  |
|---|---|
| 5v  | vcc  |
| GND | gnd  |
| SCL (13/A5) | white  |
| SDA (12/A4) | yellow  |


## Code

```cpp
#include <Arduino.h>
#include <Wire.h>

#define BYTES_TO_READ 2
#define ADDRESS 0x66


void setup()
{
	Wire.begin();
	Serial.begin(9600);
	Serial.println("Init");

}


void loop()
{
	Wire.requestFrom(ADDRESS, BYTES_TO_READ);
	
	if (Wire.available() >= BYTES_TO_READ)
	{
		int byteH = Wire.read();
		int byteL = Wire.read();
		int distanceInCM = byteH * 256 + byteL;
		
		Serial.print(distanceInCM);
		Serial.println(" cm");
	}
	
	delay(100);
}
```

!!! note ""
    source:
     [lightware github](https://github.com/LightWare-Optoelectronics/SampleLibrary/blob/master/generic_arduino_i2c/generic_arduino_i2c.ino)
     

## Convert to PWM using Servo library

![alt text](image.png)

---



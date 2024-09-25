---
tags:
    - esp32
    - arduino
    - arduino-cli
    - imu
    - mpu6050
---

## MPU6050

Read mpu-6050 data using predefined lib [ElectronicCats/mpu6050](https://github.com/ElectronicCats/mpu6050)

## Library

```bash title="search"
arduino-cli lib search MPU6050 | grep -i adafruit
#
Name: "Adafruit MPU6050"
  Author: Adafruit
  Maintainer: Adafruit <info@adafruit.com>
  Sentence: Arduino library for the MPU6050 sensors in the Adafruit shop
  Paragraph: Arduino library for the MPU6050 sensors in the Adafruit shop
  Website: https://github.com/adafruit/Adafruit_MPU6050
  Dependencies: Adafruit BusIO, Adafruit Unified Sensor, Adafruit GFX Library, Adafruit SSD1306
  Sentence: Control brushed DC motors by PWM and uses optional attached encoders to drive fixed distances. For L298 or TB6612, or Adafruit Motor Shield<br/>
```

```bash title="install"
arduino-cli lib install "Adafruit MPU6050"
```


## hello mpu

```cpp
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
}

void loop() {}
```

!!! tip c_cpp_properties.json
    ```json
    {
	"configurations": [
		{
			"name": "Linux",
			"includePath": [
				"~/.arduino15/packages/esp32/hardware/esp32/3.0.4/cores/esp32/**",
				"~/.arduino15/packages/esp32/hardware/esp32/3.0.4/libraries/**",
				"~/.arduino15/packages/esp32/hardware/esp32/3.0.4/variants/d1_mini32",
				"~/.arduino15/packages/esp32/tools/esp32-arduino-libs/idf-release_v5.1-b6b4727c58/esp32/**",
				"~/.arduino15/packages/esp32/tools/esp32-arduino-libs/idf-release_v5.1-b6b4727c58/esp32/dio_qspi/include",
				"~/Arduino/libraries/**/**"

			],

			"compilerPath": "~/.arduino15/packages/esp32/tools/esp-x32/2302/bin/xtensa-esp32-elf-g++",
			"cStandard": "c11",
			"cppStandard": "c++17"

		}

	],
	"version": 4
}
    ```
     
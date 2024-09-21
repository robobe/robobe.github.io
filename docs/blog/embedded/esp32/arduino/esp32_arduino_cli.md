---
tags:
    - esp32
    - arduino
    - cli
    - vscode
---

Init build and upload using arduino-cli

## Test using UNO
Using **arduino uno** try simple blink sketch

!!! warning Unknown FQBN and core using chinese board/ core board dependencies

    When run command `arduino-cli board list` it's return **Unknown** for FQBN and Core

    ```bash
    arduino-cli board list
    #
    Port         Protocol Type              Board Name FQBN Core
    /dev/ttyUSB0 serial   Serial Port (USB) Unknown
    ```

    even when run command `arduino-cli board listall` no **uno** family board found

    #### Install core

    ```bash
    arduino-cli core search uno
    #
    ID                  Version Name
    arduino:avr         1.8.6   Arduino AVR Boards
    arduino:megaavr     1.8.8   Arduino megaAVR Boards
    arduino:renesas_uno 1.2.1   Arduino UNO R4 Boards
    
    
    # install
    arduino-cli core install arduino:avr
    ```

    Now there avr board when we run `arduino-cli board listall | grep avr`


### Create sketch

```bash
arduino-cli sketch new Blink
cd Blink/
```
     
```cpp title="Blink.ino"
void setup() {
    pinMode(13, OUTPUT);
}

void loop() {
    digitalWrite(13, HIGH);
    delay(500);
    digitalWrite(13, LOW);
    delay(500);    
}
```

### Compile and Upload
From `Blink` parent folder

```bash title="compile"
arduino-cli compile -b arduino:avr:uno Blink -v --output-dir .
```

```bash title="upload"
arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:uno Blink -v
```

```bash title="compile and upload in one line"
arduino-cli compile --upload -p /dev/ttyUSB0 --fqbn arduino:avr:uno Blink -v
```

### Create config file

TODO

---

## ESP32

```bash
arduino-cli board list
#
Port         Protocol Type              Board Name FQBN Core
/dev/ttyUSB0 serial   Serial Port (USB) Unknown
```

!!! warning Unknown FQBN and core using chinese board/ core board dependencies
    Check UNO description above



```bash
arduino-cli core search esp32
#
ID            Version Name
arduino:esp32 2.0.17  Arduino ESP32 Boards
esp32:esp32   3.0.4   esp32
```

```bash
arduino-cli core install esp32:esp32
```

```bash
arduino-cli compile --upload -p /dev/ttyUSB0 --fqbn esp32:esp32:esp32  Blink -v
```

```bash
arduino-cli monitor -p /dev/ttyUSB0 -b esp32:esp32:esp32 --config 115200
```

### Serial monitor

```cpp
int counter = 0;
void setup() {
    Serial.begin(115200);           // set up Serial library at 9600 bps
    Serial.println("Hello world!");
}

void loop() {
    delay(500);   
    Serial.println((String)"Hello world! " +  counter); 
    counter++;
}

```

#### Monitor

```bash
arduino-cli monitor -p /dev/ttyUSB0 -b esp32:esp32:esp32 --config 115200
#
Monitor port settings:
  baudrate=115200
  bits=8
  dtr=off
  parity=none
  rts=off
  stop_bits=1

Connecting to /dev/ttyUSB0. Press CTRL-C to exit.
Hello world! 6
Hello world! 7
```

---

## VSCode
Config vscode to use `arduino-cli` and `c_cpp_properties.json` settings


```title="demo project"
.
├── Blink
│   └── Blink.ino
└── .vscode
    ├── c_cpp_properties.json
    ├── settings.json
    └── tasks.json
```

```bash
sudo apt-get install libc6-dev-i386
```

```json title="c_cpp_properties.json"
{
	"configurations": [
		{
			"name": "Linux",
			"includePath": [
				"${workspaceFolder}/Blink",
				"~/.arduino15/packages/esp32/hardware/esp32/3.0.4/cores/esp32/**",
				"~/.arduino15/packages/esp32/hardware/esp32/3.0.4/libraries/**",
				"~/.arduino15/packages/esp32/hardware/esp32/3.0.4/variants/d1_mini32",
				"~/.arduino15/packages/esp32/tools/esp32-arduino-libs/idf-release_v5.1-b6b4727c58/esp32/**",
				"~/.arduino15/packages/esp32/tools/esp32-arduino-libs/idf-release_v5.1-b6b4727c58/esp32/dio_qspi/include",
				"~/Arduino/libraries/**",
				"/usr/include/x86_64-linux-gnu/gnu/**"

			],

			"compilerPath": "~/.arduino15/packages/esp32/tools/esp-x32/2302/bin/xtensa-esp32-elf-g++",
			"cStandard": "c11",
			"cppStandard": "c++17"

		}

	],
	"version": 4
}
```

```cpp title=""Blink.io"
#include <Arduino.h>

int counter = 0;
void setup() {
    Serial.begin(115200);           // set up Serial library at 9600 bps
    Serial.println("Hello world!");
}

void loop() {
    delay(500);   
    Serial.println((String)"Hello world! " +  counter); 
    counter++;
}
```

```json title=".vscode/tasks.json"
{
    "version": "2.0.0",
    "tasks": [
        {
            "label": "build and upload",
            "type": "shell",
            "command": "arduino-cli compile --upload -p /dev/ttyUSB0 --fqbn esp32:esp32:esp32 Blink -v",
            "group": "build",
            "isBackground": false,
            "presentation": {
                "echo": true,
                "reveal": "always",
                "focus": false,
                "panel": "shared",
                "showReuseMessage": true
            },
            "problemMatcher": []
        }
    ]
}
```

---

## Reference
- [Arduino CLI: Getting Started](https://youtu.be/J-qGn1eEidA)
- [Arduino-cli - Uploading to Unknown Chinese Arduino Boards using the Arduino Command Line Interface](https://www.unix.com/programming/283468-arduino-cli-uploading-unknown-chinese-arduino-boards-using-arduino-command-line-interface.html)
- [Demo project](https://github.com/okalachev/flix/tree/master)

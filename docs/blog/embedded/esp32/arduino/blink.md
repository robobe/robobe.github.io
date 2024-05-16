---
tags:
    - esp32
    - arduino
    - platformio
    - hello world
    - blink
    - simulation
---
# Blink
ESP32 Arduino hello blink using simulation and hardware
## Hardware
### ESP32-WROOM-32 
[ESP32-WROOM-32 Pinout Reference](https://lastminuteengineers.com/esp32-wroom-32-pinout-reference/)

![](images/esp32_wroom32_pinout.png)



---

## Simulation
[Wokwi Simulation IoT project ](https://wokwi.com/) using vscode or browser


![](images/vscode_wokwi_ext.png)


![](images/wokwi_request_for_licence.png)
```cpp
#include <Arduino.h>

// Set LED_BUILTIN if it is not defined by Arduino framework
#ifndef LED_BUILTIN
    #define LED_BUILTIN 2
#endif

void setup()
{
  // initialize LED digital pin as an output.
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop()
{
  // turn the LED on (HIGH is the voltage level)
  digitalWrite(LED_BUILTIN, HIGH);
  // wait for a second
  delay(1000);
  // turn the LED off by making the voltage LOW
  digitalWrite(LED_BUILTIN, LOW);
   // wait for a second
  delay(1000);
}
```
---
tags:
    - arduino
    - blink
    - nano
    - vscode
    - platformio
---



# Blink

```cpp
#include <Arduino.h>

int led_pin=13;
void setup()
{
  pinMode(led_pin,OUTPUT); 
}

void loop()
{
  digitalWrite(led_pin, HIGH);
  delay(500);

  digitalWrite(led_pin, LOW);

  delay(500);
}
```

```ini name="platformio"
[env:nanoatmega328]
platform = atmelavr
board = nanoatmega328
framework = arduino
```
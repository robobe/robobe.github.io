---
tags:
    - arduino
    - platformio
    - libs
    - vscode
---

# Arduino project with multiple file and custom libs

## Multiple files

### project structure
```
.
├── include
│   ├── code.h
│   └── README
├── lib
│   └── README
├── platformio.ini
├── src
│   ├── code.cpp
│   └── main.cpp
└── test
    └── README
```

```cpp title="include/code.h"
#pragma once

extern char* myName;

void my_name(char* name);
```

```cpp title="src/code.cpp"
#include <Arduino.h>
#include "code.h"


void my_name(char* name){
  Serial.print(name);
}
```

```cpp title="src/main.cpp"
#include <Arduino.h>
#include "code.h"

void setup() {
  Serial.begin(9600);
  my_name("hello");
}

// the loop function runs over and over again forever
void loop() {
}
```


```ini title="platformio.ini"
[env:nanoatmega328]
platform = atmelavr
board = nanoatmega328
framework = arduino
```

---

## Using libs

!!! note Check lib folder README
     

```
.
├── include
│   └── README
├── lib
│   ├── README
│   └── SayHello
│       └── src
│           ├── SayHello.cpp
│           └── SayHello.h
├── platformio.ini
├── src
│   └── main.cpp
└── test
    └── README
```

```cpp title="lib/SayHello/src/SayHello.h"
#pragma once
#include <Arduino.h>


extern char* myName;

void say_hello(char* name);
```

```cpp title="lib/SayHello/src/SayHello.cpp"
#include "SayHello.h"

void say_hello(char* name){
    Serial.println(name);
}
```

```cpp title="src/main.cpp"
#include <Arduino.h>
#include "SayHello.h"

void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  say_hello("hello from lib");
}

void loop() {

}
```
---

## Reference
- [
Creating Libraries for PlatformIO ](https://youtu.be/3D8M_9mxGJc)
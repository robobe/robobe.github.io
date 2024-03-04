---
tags:
    - esp32
    - rtos
    - vscode
    - dev
---

# ESP32 VSCode devcontainer setup
Setup ESP32 **RTOS** dev environment using vscode and docker


```
├── CMakeLists.txt
├── .devcontainer
│   ├── devcontainer.json
│   └── Dockerfile
├── main
│   ├── app.c
│   └── CMakeLists.txt
├── sdkconfig
└── .vscode
    ├── c_cpp_properties.json
    └── settings.json
```

## devcontainer

!!! note "idf version"

    ```Docker
    FROM espressif/idf:v5.2
    ```

    The pythonBinPath change between versions

    ```
    "idf.pythonBinPath": "/opt/esp/python_env/idf5.2_py3.10_env/bin/python",
    ```
     
```Dockerfile

FROM espressif/idf:v5.2
RUN echo "source /opt/esp/idf/export.sh" >> /root/.bashrc
```

```json title="devcontainer.json"
{
	"name": "ESP-IDF",
	"build": {
		"dockerfile": "Dockerfile"
	},
    "customizations": {
        "vscode": {
            "settings": {
                "idf.showOnboardingOnInit": false,
                "idf.toolsPath": "/opt/esp",
                "idf.espIdfPath": "/opt/esp/idf",
                "idf.pythonBinPath": "/opt/esp/python_env/idf5.2_py3.10_env/bin/python",
                "idf.port": "/dev/ttyUSB0",
                "terminal.integrated.shell.linux": "/bin/bash"
            },
            // Install relevant extensions in the dev container
            "extensions": [
                "espressif.esp-idf-extension",
                "ms-vscode.cmake-tools",
                "ms-vscode.cpptools",
                "albert.tabout"
            ]
        }
    },
	// Pre-configure ESP-IDF extension
	
	"userEnvProbe": "loginInteractiveShell",
	// Forward serial device to enable flash & monitor
	"runArgs": [
		"--device=/dev/ttyUSB0"
	]
}
```


### .vscode

```json title="c_cpp_properties.json"
{
    "configurations": [
        {
            "name": "ESP-IDF",
            "compilerPath": "/opt/esp/tools/xtensa-esp-elf/esp-13.2.0_20230928/xtensa-esp-elf/bin/xtensa-esp32-elf-gcc",
            "cStandard": "c11",
            "cppStandard": "c++17",
            "includePath": [
                "${config:idf.espIdfPath}/components/**",
                "${config:idf.espIdfPathWin}/components/**",
                "${config:idf.espAdfPath}/components/**",
                "${config:idf.espAdfPathWin}/components/**",
                "${workspaceFolder}/**"
            ],
            "browse": {
                "path": [
                    "${config:idf.espIdfPath}/components",
                    "${config:idf.espIdfPathWin}/components",
                    "${config:idf.espAdfPath}/components/**",
                    "${config:idf.espAdfPathWin}/components/**",
                    "${workspaceFolder}"
                ],
                "limitSymbolsToIncludedHeaders": false
            },
            "defines": [
                "CONFIG_FREERTOS_HZ=1000",
                "configTICK_RATE_HZ=1000"
            ]
        }
    ],
    "version": 4
}
```

---

### Demo

```c title="root CMakeLists.txt"
cmake_minimum_required(VERSION 3.5)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
idf_build_set_property(COMPILE_OPTIONS "-Wno-error" APPEND)

project(myProject)
```

```c title="main CMakeLists.txt"
idf_component_register(
    SRCS "app.c"
    INCLUDE_DIRS "")
```

```c title="app.c"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/gpio.h"

#define BLINK_GPIO 2

static const BaseType_t app_cpu = 0;

void blinky(void *pvParameter)
{

    esp_rom_gpio_pad_select_gpio(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
    while (1)
    {
        /* Blink off (output low) */
        gpio_set_level(BLINK_GPIO, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        /* Blink on (output high) */
        gpio_set_level(BLINK_GPIO, 1);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void app_main()
{
    // Task to run forever
    xTaskCreatePinnedToCore( // Use xTaskCreate() in vanilla FreeRTOS
        blinky,         // Function to be called
        "Toggle 1",          // Name of task
        1024,                // Stack size (bytes in ESP32, words in FreeRTOS)
        NULL,                // Parameter to pass to function
        1,                   // Task priority (0 to configMAX_PRIORITIES - 1)
        NULL,                // Task handle
        app_cpu);            // Run on one core for demo purposes (ESP32 only)
}

```
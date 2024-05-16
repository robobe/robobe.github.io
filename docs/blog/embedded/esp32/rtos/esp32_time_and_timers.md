---
tags:
    - esp32
    - rtos
    - embedded
    - time
    - timer
---

# ESP32 time and timer

## Time
- get time from boot

```c
#include <stdio.h>
#include "freertos/FreeRTOS.h"
// #include "freertos/timers.h"
#include "esp_timer.h"
#include "esp_log.h"


#define LOG_TAG "MY_LOG"


void app_main()
{
    ESP_LOGI(LOG_TAG, "hello info");
    // int64_t time_since_boot = esp_timer_get_time();
    ESP_LOGI(LOG_TAG, "timers: %lld microsecondes", esp_timer_get_time());
    
}
```

## Tick

rtos define tick as 10ms

```c
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "esp_log.h"
#include "esp_timer.h"

#define LOG_TAG "MY_LOG"

void app_main()
{

    ESP_LOGI(LOG_TAG, "tick: %lu", portTICK_PERIOD_MS);
    printf("start: %lld millisec\n", esp_timer_get_time()/1000);
    vTaskDelay(pdMS_TO_TICKS(1000));
    printf("end: %lld millisec\n", esp_timer_get_time()/1000);
}
```

```
start: 34 millisec
end: 1032 millisec
```
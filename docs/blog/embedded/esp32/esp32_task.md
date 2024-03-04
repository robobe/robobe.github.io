---
tags:
    - esp32
    - rtos
    - embedded
    - task
---

# Task

```c
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"


#define LOG_TAG "MY_LOG"

void vTask()
{
    for (size_t i=0; i< 5 ; i++)
    {
        ESP_LOGI(LOG_TAG, "%d", i);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    vTaskDelete(NULL);
}

void app_main()
{

    xTaskCreate(
        vTask,
        "task1",
        2048,
        NULL,
        1,
        NULL
    );
}


```
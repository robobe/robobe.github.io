---
tags:
    - esp32
    - rtos
    - embedded
    - semaphore
    - mutex
---


## Binary semaphore
```c
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"

#define LOG_TAG "MY_LOG"

SemaphoreHandle_t x_semaphore = NULL;


void vTask()
{
    for (size_t i=0; i< 5 ; i++)
    {
        ESP_LOGI(LOG_TAG, "%d", i);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    xSemaphoreGive(x_semaphore);
    vTaskDelete(NULL);
}

void app_main()
{
    x_semaphore = xSemaphoreCreateBinary();
    xTaskCreate(
        vTask,
        "task1",
        2048,
        NULL,
        1,
        NULL
    );

    xSemaphoreTake(x_semaphore, portMAX_DELAY);
    printf("--done--");

}

```

---

## Mutex

- xSemaphoreCreateMutex
- xSemaphoreTake / xSemaphoreGive pair

```c
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"

#define LOG_TAG "MY_LOG"

SemaphoreHandle_t x_semaphore = NULL;

void vTask()
{
    xSemaphoreTake(x_semaphore, portMAX_DELAY);
    for (size_t i = 0; i < 5; i++)
    {
        ESP_LOGI(LOG_TAG, "Task 1: %d", i);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    xSemaphoreGive(x_semaphore);

    vTaskDelete(NULL);
}

void vTask2()
{
    xSemaphoreTake(x_semaphore, portMAX_DELAY);
    for (size_t i = 0; i < 5; i++)
    {
        ESP_LOGI(LOG_TAG, "Task 2: %d", i);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    xSemaphoreGive(x_semaphore);
    vTaskDelete(NULL);
}

void app_main()
{
    x_semaphore = xSemaphoreCreateMutex();
    xTaskCreate(
        vTask,
        "task1",
        2048,
        NULL,
        1,
        NULL);

    xTaskCreate(
        vTask2,
        "task2",
        2048,
        NULL,
        1,
        NULL);
}

```
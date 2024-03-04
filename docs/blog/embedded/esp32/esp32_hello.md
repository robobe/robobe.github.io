---
tags:
    - esp32
    - rtos
    - embedded
---


# ESP rtos hello world


![](images/log_config.png)

```c
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "esp_log.h"


#define LOG_TAG "MY_LOG"


void app_main()
{
    printf("hello world\n");
    ESP_LOGI(LOG_TAG, "hello info");
    ESP_LOGW(LOG_TAG, "hello warn");
    ESP_LOGE(LOG_TAG, "hello error");
    ESP_LOGD(LOG_TAG, "hello debug");
    ESP_LOGV(LOG_TAG, "hello verbose");
}

```

![](images/log_application_output.png)
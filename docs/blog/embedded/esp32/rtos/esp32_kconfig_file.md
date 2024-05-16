---
tags:
    - esp32
    - kconfig
    - embedded
---

```c
#include <stdio.h>
#include "esp_log.h"
#include "sdkconfig.h"

#define LOG_TAG "MY_LOG"

void app_main()
{
    ESP_LOGI(LOG_TAG, "SSID: %s", CONFIG_SSID);
    ESP_LOGI(LOG_TAG, "PASS: %s", CONFIG_PASSWORD);
}
```


```bash
menu "WIFI"
    config SSID
        string "SSID"
        default "my ssid"

    config PASSWORD
        string "PASSWORD"
        default "my password"
endmenu
```


!!! tip 
    Clean and rebuild


## menuconfig

![](images/custom_kconfig_menuconfig.png)


---

## Reference
- [ESP-IDF Configuration | How to add custom configuration in project config](https://medium.com/@bhautik.markeye/esp-idf-configuration-how-to-add-custom-configuration-in-project-config-728f81b8d0d8)
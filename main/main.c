#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "smartconfig_main.h"


void app_main(void)
{
    int i = 0;
    while (i < 1) {
        printf("[%d] Hello world!\n", i);
        i++;
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    ESP_ERROR_CHECK(nvs_flash_init());
    initialise_wifi();
    printf("Hello Yangx  !\n");
}

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "smartconfig_main.h"
#include "http_request_main.h"
#include "sip_oled.h"

void app_main(void)
{
    // 打印IDF版本
    printf("SDK version:%s\n", esp_get_idf_version());
    // Hello world!
    int i = 0;
    while (i < 1) {
        printf("[%d] Hello world!\n", i);
        i++;
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    ESP_ERROR_CHECK(nvs_flash_init());
    // // 开启WiFi智能配网线程
    // initialise_wifi();
    // // 开启连接高德API线程
    // http_main();

    printf("Hello Yangx  !\n");
    // 屏幕显示
    oled_main();
}

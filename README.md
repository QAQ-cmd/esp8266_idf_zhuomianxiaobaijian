# esp8266_idf_zhuomianxiaobaijian

#### 说明：

使用乐鑫官方的RTOS做一个类似于 太空人WiFi天气时钟 项目(arduino)的桌面小摆件项目

#### 硬件：

1. 模块：esp8266
2. 屏幕：ST7735S  (深超光电液晶玻璃CT018TN01 1.77icn)

#### 连接：
* Connection:  

| Signal    | Oled   | ESP8266    |
|-----------|--------|--------    |
| 5.0V      | VCC    | VCC        |
| SCLK      | CLK    | GPIO14(D5) |
| D/C       | RS     | GPIO12(D6) |
| MOSI      | SDA    | GPIO13(D7) |
| RST       | RST    | GPIO15(D8) |
| CS        | CS     | GND        |
| GND       | GND    | GND        |

#### 引用：

[ESP8266-RTOS-SDK](https://github.com/espressif/ESP8266_RTOS_SDK/tree/master) 是乐鑫官方推出的物联网开发框架

[esp-idf-template](https://github.com/espressif/esp-idf-template) 初始化仓库模板

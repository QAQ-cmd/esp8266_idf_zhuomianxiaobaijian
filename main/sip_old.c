/* spi_oled example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp8266/gpio_struct.h"
#include "esp8266/spi_struct.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_libc.h"

#include "driver/gpio.h"
#include "driver/spi.h"
#include "sip_old.h"

static const char *TAG = "spi_oled";

#define OLED_DC_GPIO     12
#define OLED_RST_GPIO    15
#define OLED_PIN_SEL  (1ULL<<OLED_DC_GPIO) | (1ULL<<OLED_RST_GPIO)

static uint8_t oled_dc_level = 0;

//定义LCD的尺寸
#define LCD_W 128
#define LCD_H 160

//配置LCD的长宽给reg
#define SETXCMD 0x2A
#define SETYCMD 0x2B

//开始写入GRAM reg
#define WRAMCMD 0x2C

//画笔颜色
#define POINT_COLOR 0x0000
//背景色
#define BACK_COLOR 0xFFFF 

#define GREEN 0x07E0

static esp_err_t oled_delay_ms(uint32_t time)
{
    vTaskDelay(time / portTICK_RATE_MS);
    return ESP_OK;
}

static esp_err_t oled_set_dc(uint8_t dc)
{
    oled_dc_level = dc;
    return ESP_OK;
}

// Write an 8-bit cmd
static esp_err_t oled_write_cmd(uint8_t data)
{
    uint32_t buf = data << 24; // In order to improve the transmission efficiency, it is recommended that the external incoming data is (uint32_t *) type data, do not use other type data.
    spi_trans_t trans = {0};
    trans.mosi = &buf;
    trans.bits.mosi = 8;
    oled_set_dc(0);
    spi_trans(HSPI_HOST, &trans);
    return ESP_OK;
}

// Write an 8-bit data
static esp_err_t oled_write_data(uint8_t data)
{
    uint32_t buf = data << 24; // In order to improve the transmission efficiency, it is recommended that the external incoming data is (uint32_t *) type data, do not use other type data.
    spi_trans_t trans = {0};
    trans.mosi = &buf;
    trans.bits.mosi = 8;
    oled_set_dc(1);
    spi_trans(HSPI_HOST, &trans);
    return ESP_OK;
}

// write cmd and data
static esp_err_t oled_writereg(uint8_t data0, uint8_t data)
{
    oled_write_cmd(data0);
    oled_write_data(data);
    return ESP_OK;
}

// Write an 16-bit data
static esp_err_t oled_w_data_16bit(uint16_t data)
{
    uint8_t x;
    uint32_t buf[2];
    spi_trans_t trans = {0};
    trans.mosi = buf;
    trans.bits.mosi = 2 * 8;

    for (x = 0; x < 2; x++) {
        buf[x] = data >> 8 | data;
    }

    // SPI transfers 2 bytes at a time, transmits twice, increasing the screen refresh rate
    for (x = 0; x < 2; x++) {
        // 设置像素的起始位
        // oled_set_pos(0, x);
        oled_set_dc(1);
        spi_trans(HSPI_HOST, &trans);
        // spi_trans(HSPI_HOST, &trans);
    }

    return ESP_OK;
}


// static esp_err_t oled_w_data_16bit(uint16_t data)
// {
//     uint32_t buf = data << 16; // 将16位数据左移，使其位于32位数据的高16位
//     spi_trans_t trans = {0};
//     trans.mosi = &buf;
//     trans.bits.mosi = 16; // 设置要传输的数据位数为16位
//     oled_set_dc(1); // 设置为数据传输模式
//     spi_trans(HSPI_HOST, &trans); // 通过SPI传输
//     return ESP_OK;
// }


static esp_err_t oled_rst()
{
    gpio_set_level(OLED_RST_GPIO, 1);
    oled_delay_ms(50);
    gpio_set_level(OLED_RST_GPIO, 0);
    oled_delay_ms(50);
    gpio_set_level(OLED_RST_GPIO, 1);
    oled_delay_ms(50);
    return ESP_OK;
}

static esp_err_t oled_init()
{
    oled_rst(); // Reset OLED
 //************* ST7735S初始化**********//	
	oled_write_cmd(0x11);//Sleep exit 
    oled_delay_ms(120);
	//ST7735R Frame Rate
	oled_write_cmd(0xB1); 
	oled_write_data(0x01); 
	oled_write_data(0x2C); 
	oled_write_data(0x2D); 
	oled_write_cmd(0xB2); 
	oled_write_data(0x01); 
	oled_write_data(0x2C); 
	oled_write_data(0x2D); 
	oled_write_cmd(0xB3); 
	oled_write_data(0x01); 
	oled_write_data(0x2C); 
	oled_write_data(0x2D); 
	oled_write_data(0x01); 
	oled_write_data(0x2C); 
	oled_write_data(0x2D); 	
	oled_write_cmd(0xB4); //Column inversion 
	oled_write_data(0x07); 	
	//ST7735R Power Sequence
	oled_write_cmd(0xC0); 
	oled_write_data(0xA2); 
	oled_write_data(0x02); 
	oled_write_data(0x84); 
	oled_write_cmd(0xC1); 
	oled_write_data(0xC5); 
	oled_write_cmd(0xC2); 
	oled_write_data(0x0A); 
	oled_write_data(0x00); 
	oled_write_cmd(0xC3); 
	oled_write_data(0x8A); 
	oled_write_data(0x2A); 
	oled_write_cmd(0xC4); 
	oled_write_data(0x8A); 
	oled_write_data(0xEE); 
	oled_write_cmd(0xC5); //VCOM 
	oled_write_data(0x0E); 	
	oled_write_cmd(0x36); //MX, MY, RGB mode 
	oled_write_data(0xC0); 
	//ST7735R Gamma Sequence
	oled_write_cmd(0xe0); 
	oled_write_data(0x0f); 
	oled_write_data(0x1a); 
	oled_write_data(0x0f); 
	oled_write_data(0x18); 
	oled_write_data(0x2f); 
	oled_write_data(0x28); 
	oled_write_data(0x20); 
	oled_write_data(0x22); 
	oled_write_data(0x1f); 
	oled_write_data(0x1b); 
	oled_write_data(0x23); 
	oled_write_data(0x37); 
	oled_write_data(0x00); 	
	oled_write_data(0x07); 
	oled_write_data(0x02); 
	oled_write_data(0x10); 
	oled_write_cmd(0xe1); 
	oled_write_data(0x0f); 
	oled_write_data(0x1b); 
	oled_write_data(0x0f); 
	oled_write_data(0x17); 
	oled_write_data(0x33); 
	oled_write_data(0x2c); 
	oled_write_data(0x29); 
	oled_write_data(0x2e); 
	oled_write_data(0x30); 
	oled_write_data(0x30); 
	oled_write_data(0x39); 
	oled_write_data(0x3f); 
	oled_write_data(0x00); 
	oled_write_data(0x07); 
	oled_write_data(0x03); 
	oled_write_data(0x10);  	
	oled_write_cmd(0x2a);
	oled_write_data(0x00);
	oled_write_data(0x00);
	oled_write_data(0x00);
	oled_write_data(0x7f);
	oled_write_cmd(0x2b);
	oled_write_data(0x00);
	oled_write_data(0x00);
	oled_write_data(0x00);
	oled_write_data(0x9f);
	oled_write_cmd(0xF0); //Enable test command  
	oled_write_data(0x01); 
	oled_write_cmd(0xF6); //Disable ram power save mode 
	oled_write_data(0x00); 	
	oled_write_cmd(0x3A); //65k mode 
	oled_write_data(0x05); 
	oled_write_cmd(0x29); //display on
    oled_writereg(0x36,(0<<3)|(1<<6)|(1<<7));//BGR==1,MY==0,MX==0,MV==0
    return ESP_OK;
}

// static esp_err_t oled_set_pos(uint8_t x_start, uint8_t y_start)
// {
//     oled_write_cmd(0xb0 + y_start);
//     oled_write_cmd(((x_start & 0xf0) >> 4) | 0x10);
//     oled_write_cmd((x_start & 0x0f) | 0x01);
//     return ESP_OK;
// }

// static esp_err_t oled_clear(uint8_t data)
// {
//     uint8_t x;
//     uint32_t buf[16];
//     spi_trans_t trans = {0};
//     trans.mosi = buf;
//     trans.bits.mosi = 64 * 8;

//     for (x = 0; x < 16; x++) {
//         buf[x] = data << 24 | data << 16 | data << 8 | data;
//     }

//     // SPI transfers 64 bytes at a time, transmits twice, increasing the screen refresh rate
//     for (x = 0; x < 8; x++) {
//         oled_set_pos(0, x);
//         oled_set_dc(1);
//         spi_trans(HSPI_HOST, &trans);
//         spi_trans(HSPI_HOST, &trans);
//     }

//     return ESP_OK;
// }

static void IRAM_ATTR spi_event_callback(int event, void *arg)
{
    switch (event) {
        case SPI_INIT_EVENT: {

        }
        break;

        case SPI_TRANS_START_EVENT: {
            gpio_set_level(OLED_DC_GPIO, oled_dc_level);
        }
        break;

        case SPI_TRANS_DONE_EVENT: {

        }
        break;

        case SPI_DEINIT_EVENT: {
        }
        break;
    }
}

static void oled_setwindows(uint8_t xStar, uint8_t yStar,uint8_t xEnd,uint8_t yEnd)
{	
	oled_write_cmd(SETXCMD);	
	oled_write_data(0x00);
	oled_write_data(xStar);		
	oled_write_data(0x00);
	oled_write_data(xEnd);

	oled_write_cmd(SETYCMD);
	oled_write_data(0x00);
	oled_write_data(yStar);		
	oled_write_data(0x00);
	oled_write_data(yEnd);

	oled_write_cmd(WRAMCMD);		        //开始写入GRAM				
} 

// static void oled_drawpoint(uint8_t x,uint8_t y)
// {
// 	oled_setwindows(x,y,x,y);               //设置光标位置
// 	oled_w_data_16bit(POINT_COLOR); 	    
// } 

void oled_fill(uint8_t sx,uint8_t sy,uint8_t ex,uint8_t ey,uint16_t color)
{  	
	uint8_t i,j;			
	uint8_t width=ex-sx+1; 		            //得到填充的宽度
	uint8_t height=ey-sy+1;		            //高度
	oled_setwindows(sx,sy,ex,ey);            //设置显示窗口
	for( i=0 ; i<height ; i++ )
	{
		for( j=0 ; j<width ; j++ )
		oled_w_data_16bit(color);	        //写入数据
        ESP_LOGI(TAG, "yangx ----write data color\n");
	}

	oled_setwindows( 0 , 0 , LCD_W - 1 , LCD_H - 1 );//恢复窗口设置为全屏
}

void oled_main(void)
{
    // uint8_t x = 0;

    ESP_LOGI(TAG, "init gpio ");
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = OLED_PIN_SEL;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    ESP_LOGI(TAG, "init hspi ");
    spi_config_t spi_config;
    // Load default interface parameters
    // CS_EN:1, MISO_EN:1, MOSI_EN:1, BYTE_TX_ORDER:1, BYTE_TX_ORDER:1, BIT_RX_ORDER:0, BIT_TX_ORDER:0, CPHA:0, CPOL:0
    spi_config.interface.val = SPI_DEFAULT_INTERFACE;
    // Load default interrupt enable
    // TRANS_DONE: true, WRITE_STATUS: false, READ_STATUS: false, WRITE_BUFFER: false, READ_BUFFER: false
    spi_config.intr_enable.val = SPI_MASTER_DEFAULT_INTR_ENABLE;
    // Cancel hardware cs
    spi_config.interface.cs_en = 0;
    // MISO pin is used for DC
    spi_config.interface.miso_en = 0;
    // CPOL: 0, CPHA: 0
    spi_config.interface.cpol = 0;
    spi_config.interface.cpha = 0;
    // Set SPI to master mode
    // 8266 Only support half-duplex
    spi_config.mode = SPI_MASTER_MODE;
    // Set the SPI clock frequency division factor
    spi_config.clk_div = SPI_2MHz_DIV;
    // Register SPI event callback function
    spi_config.event_cb = spi_event_callback;
    spi_init(HSPI_HOST, &spi_config);

    ESP_LOGI(TAG, "init oled ");
    oled_init();
    // oled_clear(0x00);
    // 矩形方框
    // 画点
    // oled_drawpoint(40, 40);
    // 纯色填充
    oled_fill(0 ,0 ,LCD_W ,LCD_H ,GREEN);

    // while (1) {
    //     // oled_clear(x);
    //     // oled_drawpoint(x, 40);
    //     oled_delay_ms(1000);
    //     // x++;
    // }
}
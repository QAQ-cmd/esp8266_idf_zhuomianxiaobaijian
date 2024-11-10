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
#include "sip_oled.h"

static const char *TAG = "spi_oled";

#define OLED_CLK_GPIO     14
#define OLED_RST_GPIO    15
#define OLED_CS_GPIO      3
#define OLED_SDA_GPIO    13
#define OLED_RS_GPIO     12
#define OLED_PIN_SEL  ((1ULL<<OLED_CLK_GPIO) | (1ULL<<OLED_RST_GPIO) | (1ULL<<OLED_CS_GPIO) | (1ULL<<OLED_SDA_GPIO) | (1ULL<<OLED_RS_GPIO))

#define GPIO_OUTPUT_PIN_SEL  (1ULL<<GPIO_OUTPUT_IO_0)

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

#define GBLUE			 0X07FF
#define RED           	 0xF800
#define GREEN            0x07E0
#define BLUE         	 0x001F 

static esp_err_t oled_delay_ms(uint32_t time)
{
    vTaskDelay(time / portTICK_RATE_MS);
    return ESP_OK;
}

void spi_write_byte(uint8_t data)
{
	uint8_t val=0x80;
	while(val)
	{
		if(data&val)
		{
            gpio_set_level(OLED_SDA_GPIO,1);
		}
		else
		{
            gpio_set_level(OLED_SDA_GPIO,0);
		}
        gpio_set_level(OLED_CLK_GPIO,0);
        gpio_set_level(OLED_CLK_GPIO,1);
		val>>=1;
	}
}


// Write an 8-bit cmd
static esp_err_t oled_write_cmd(uint8_t data)
{
    gpio_set_level(OLED_RS_GPIO,0);
    gpio_set_level(OLED_CS_GPIO,0);
    spi_write_byte(data);
    gpio_set_level(OLED_CS_GPIO,1);
    return ESP_OK;
}

// Write an 8-bit data
static esp_err_t oled_write_data(uint8_t data)
{
    gpio_set_level(OLED_RS_GPIO,1);
    gpio_set_level(OLED_CS_GPIO,0);
    spi_write_byte(data);
    gpio_set_level(OLED_CS_GPIO,1);
    return ESP_OK;
}

// write data 16-bit
static esp_err_t oled_write_data_16Bit(uint16_t data)
{
    gpio_set_level(OLED_RS_GPIO,1);
    gpio_set_level(OLED_CS_GPIO,0);
    spi_write_byte(data>>8);
    spi_write_byte(data);
    gpio_set_level(OLED_CS_GPIO,1);
    return ESP_OK;
}

// write cmd and data
static esp_err_t oled_writereg(uint8_t data0, uint8_t data)
{
    oled_write_cmd(data0);
    oled_write_data(data);
    return ESP_OK;
}

// reset oled
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
	oled_delay_ms(120);
	//设置LCD属性参数
    oled_writereg(0x36,(0<<3)|(1<<6)|(1<<7));//BGR==1,MY==0,MX==0,MV==0
    return ESP_OK;
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

void oled_fill(uint8_t sx,uint8_t sy,uint8_t ex,uint8_t ey,uint16_t color)
{  	
	uint8_t i,j;			
	uint8_t width=ex-sx+1; 		            //得到填充的宽度
	uint8_t height=ey-sy+1;		            //高度
	// oled_setwindows(sx,sy,ex,ey);            //设置显示窗口

	for( i=0 ; i<width ; i++ )
	{
		for( j=0 ; j<height ; j++ )
        {
        oled_write_data_16Bit(color);	        //写入数据
        }
	}
	oled_setwindows( 0 , 0 , LCD_W - 1 , LCD_H - 1 );
}

void oled_main(void)
{

    ESP_LOGI(TAG, "init gpio ");
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = OLED_PIN_SEL;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    ESP_LOGI(TAG, "init oled ");
    oled_init();

    // 纯色填充
	while(1)
	{	
		oled_fill(0 ,0 ,LCD_W ,LCD_H ,RED);
        oled_delay_ms(100);	
		oled_fill(0 ,0 ,LCD_W ,LCD_H ,GREEN);
        oled_delay_ms(100);	
		oled_fill(0 ,0 ,LCD_W ,LCD_H ,BLUE);
        oled_delay_ms(100);
	}


}
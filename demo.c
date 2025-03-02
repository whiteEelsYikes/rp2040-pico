#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/dma.h"
#include "hardware/timer.h"
#include "hardware/uart.h"

// #include "oled_iic/bsp_iic_oled.h"

// #include "oled/OLED.h"
// #include "oled/OLED_Data.h"

// #include "lvgl/lvgl.h"

// #include "ui/ui.h"

// I2C defines
// This example will use I2C0 on GPIO8 (SDA) and GPIO9 (SCL) running at 400KHz.
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define I2C_PORT i2c0
#define I2C_SDA 8
#define I2C_SCL 9

// Data will be copied from src to dst
const char src[] = "Hello, world! (from DMA)";
char dst[count_of(src)];

int64_t alarm_callback(alarm_id_t id, void *user_data) {
    // Put your timeout handler code in here
    return 0;
}

// UART defines
// By default the stdout UART is `uart0`, so we will use the second one
#define UART_ID uart1
#define BAUD_RATE 115200

// Use pins 4 and 5 for UART1
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define UART_TX_PIN 4
#define UART_RX_PIN 5



// int main()
// {
//     stdio_init_all();

//     // I2C Initialisation. Using it at 400Khz.
//     i2c_init(I2C_PORT, 400*1000);
    
//     gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
//     gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
//     gpio_pull_up(I2C_SDA);
//     gpio_pull_up(I2C_SCL);
//     // For more examples of I2C use see https://github.com/raspberrypi/pico-examples/tree/master/i2c

//     // Get a free channel, panic() if there are none
//     int chan = dma_claim_unused_channel(true);
    
//     // 8 bit transfers. Both read and write address increment after each
//     // transfer (each pointing to a location in src or dst respectively).
//     // No DREQ is selected, so the DMA transfers as fast as it can.
    
//     dma_channel_config c = dma_channel_get_default_config(chan);
//     channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
//     channel_config_set_read_increment(&c, true);
//     channel_config_set_write_increment(&c, true);
    
//     dma_channel_configure(
//         chan,          // Channel to be configured
//         &c,            // The configuration we just created
//         dst,           // The initial write address
//         src,           // The initial read address
//         count_of(src), // Number of transfers; in this case each is 1 byte.
//         true           // Start immediately.
//     );
    
//     // We could choose to go and do something else whilst the DMA is doing its
//     // thing. In this case the processor has nothing else to do, so we just
//     // wait for the DMA to finish.
//     dma_channel_wait_for_finish_blocking(chan);
    
//     // The DMA has now copied our text from the transmit buffer (src) to the
//     // receive buffer (dst), so we can print it out from there.
//     puts(dst);

//     // Timer example code - This example fires off the callback after 2000ms
//     add_alarm_in_ms(2000, alarm_callback, NULL, false);
//     // For more examples of timer use see https://github.com/raspberrypi/pico-examples/tree/master/timer

//     // Set up our UART
//     uart_init(UART_ID, BAUD_RATE);
//     // Set the TX and RX pins by using the function select on the GPIO
//     // Set datasheet for more information on function select
//     gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
//     gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    
//     // Use some the various UART functions to send out data
//     // In a default system, printf will also output via the default UART
    
//     // Send out a string, with CR/LF conversions
//     uart_puts(UART_ID, " Hello, UART!\n");
    
//     // For more examples of UART use see https://github.com/raspberrypi/pico-examples/tree/master/uart

//     while (true) {
//         printf("Hello, world!\n");
//         sleep_ms(1000);
//     }
// }



// int main() {
//     stdio_init_all();
//     Bsp_IIC_Init();
//     OLED_Init();
//     OLED_CLS();
//     OLED_ShowStr(0, 0, "Hello, Pico!", 2);
//     while (1) {
//         // 主循环
//     }
//     return 0;
// }











// void my_disp_flush(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p) {
//     uint8_t x_start = (uint8_t)area->x1;
//     uint8_t x_end = (uint8_t)area->x2;
//     uint8_t y_start = (uint8_t)area->y1;
//     uint8_t y_end = (uint8_t)area->y2;

//     // 设置显示范围
//     OLED_WriteCommand(0x21); // Set column address
//     OLED_WriteCommand(x_start);
//     OLED_WriteCommand(x_end);

//     OLED_WriteCommand(0x22); // Set page address
//     OLED_WriteCommand(y_start / 8); // 转换为页地址
//     OLED_WriteCommand(y_end / 8);

//     // 将像素数据发送到显示屏
//     for (uint8_t y = y_start / 8; y <= y_end / 8; y++) {
//         for (uint8_t x = x_start; x <= x_end; x++) {
//             uint8_t pixel_data = 0;
//             for (uint8_t bit = 0; bit < 8; bit++) {
//                 if (lv_color_brightness(*color_p) > 128) {
//                     pixel_data |= (1 << bit);
//                 }
//                 color_p++;
//             }
//             OLED_WriteData(&pixel_data, 1);
//         }
//     }

//     // 通知LVGL刷新完成
//     lv_disp_flush_ready(disp_drv);
// }









// #include "lvgl.h"
// #include "hardware/i2c.h"


// void my_disp_flush(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p) {
//     int32_t x, y;

//     // 遍历显示区域，设置每个像素值
//     for(y = area->y1; y <= area->y2; y++) {
//         for(x = area->x1; x <= area->x2; x++) {
//             // 将lv_color_t的颜色值转换为单色值
//             uint8_t value = (lv_color_brightness(*color_p) > 128) ? 0x01 : 0x00;
//             OLED_DrawPoint_Plus(x, y, value);
//             color_p++;
//         }
//     }

//     // 通知LVGL刷新完成
//     OLED_Printf(0, 55, OLED_6X8, "%d.%d.%d-%s", lv_version_major(),lv_version_minor(), lv_version_patch(), lv_version_info());
// 	OLED_Update();
// 	lv_disp_flush_ready(disp_drv);
// }

















// // static uint32_t my_tick_get_cb (void) { return millis(); }



// void DEV_Delay_ms(uint32_t xms)
// {
//     sleep_ms(xms);
// }




// #define FRAME_TIME	33
// void main_loop()
// {
// 	uint64_t ms_last = time_us_64() / 1000;
// 	int count = 0;
//     //UWORD 图像[LCD_1IN28_WIDTH*10];
// 	uint32_t start_time_ms = 0;
// 	bool bg_init_finish = false;
// 	while( 1 )
// 	{
// 		//Block mutex();
// 		uint64_t ms_now = time_us_64() / 1000;
// 		uint64_t ms_loop_start = ms_now;
// 		uint32_t ms_delta = (uint32_t)(ms_now - ms_last);
// 		ms_last = ms_now;

// 		// 初始化 bg
// 		// start_time_ms+=ms_delta;
// 		// 如果（ ！bg_init_finish ）
// 		// {
// 		// 	bg_init_finish = InitBg（start_time_ms）;
// 		// 	如果 （ bg_init_finish ）
// 		// 	{
// 		// 		CreateRotator（）;
// 		// 	}
// 		// }

// 		lv_tick_inc( ms_delta );

// 		// memset（ 图像， 计数， LCD_1IN28_WIDTH*10*2）;
// 		// LCD_1IN28_SetWindows（ 0， 0， 240， 240 ）;
// 		// DEV_Digital_Write（ LCD_DC_PIN， 1 ）;
// 		// spi_set_format（ SPI_PORT， 16， SPI_CPOL_0， SPI_CPHA_0， SPI_MSB_FIRST ）;
// 		// for（ int i=0;i<24;i++）
// 		// 	spi_write16_blocking（ SPI_PORT， 图像， LCD_1IN28_WIDTH*10 ）;
// 		// spi_set_format（ SPI_PORT， 8， SPI_CPOL_0， SPI_CPHA_0， SPI_MSB_FIRST ）;

// 		uint32_t delay = lv_timer_handler();
// 		// uint32_t延迟 = FRAME_TIME;
// 		//unblock_mutex（）;
// 		//printf（“延迟 %d\n”，延迟）;
// 		//if（ （int）delay > 0 ）
// 		//	DEV_Delay_ms（ 延迟 ）;

// 		count ++;
// 		if( count < 180 )
// 		{
// 			printf("%d\n",ms_delta);
// 		}
		
// 		ms_now = time_us_64() / 1000;
// 		ms_delta = (uint32_t)(ms_now - ms_loop_start);
// 		if( FRAME_TIME > ms_delta )
// 			DEV_Delay_ms( FRAME_TIME - ms_delta );
		
// 	}
// }








//  int main(void)
//  {
//     stdio_init_all();
// 	lv_init();
//  	OLED_Init();

// 	static lv_disp_draw_buf_t lv_draw_buf;
// 	static lv_color_t lv_color_buf1[OLED_0IN96_WIDE * OLED_0IN96_HIGH];

// 	static lv_disp_drv_t disp_drv;
// 	lv_disp_draw_buf_init( &lv_draw_buf, lv_color_buf1, NULL, OLED_0IN96_WIDE * OLED_0IN96_HIGH );  
// 	lv_disp_drv_init(&disp_drv);
// 	disp_drv.flush_cb = my_disp_flush;      /*Set your driver function*/
//     // disp_drv.wait_cb = my_disp_wait;
// 	disp_drv.draw_buf = &lv_draw_buf;     /*Assign the buffer to the display*/
// 	disp_drv.hor_res = OLED_0IN96_WIDE;   /*Set the horizontal resolution of the display*/
// 	disp_drv.ver_res = OLED_0IN96_HIGH;  /*Set the vertical resolution of the display*/
//     disp_drv.direct_mode = 0;
// 	lv_disp_drv_register(&disp_drv);


// 	// lv_tick_set_cb( my_tick_get_cb );

	
// 	// lv_obj_t * ui_Screen1;
// 	// lv_obj_t * ui_Label1;
// 	// lv_obj_t * ui_Switch1;
	
// 	// lv_obj_t * ui____initial_actions0;


// 	// lv_disp_t * dispp = lv_display_get_default();
//     // lv_theme_t * theme = lv_theme_default_init(dispp, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_RED),
//     //                                            true, LV_FONT_DEFAULT);
//     // lv_disp_set_theme(dispp, theme);

// 	// ui_Screen1 = lv_obj_create(NULL);
//     // lv_obj_remove_flag(ui_Screen1, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

//     // ui_Label1 = lv_label_create(ui_Screen1);
//     // lv_obj_set_width(ui_Label1, LV_SIZE_CONTENT);   /// 1
//     // lv_obj_set_height(ui_Label1, LV_SIZE_CONTENT);    /// 1
//     // lv_obj_set_align(ui_Label1, LV_ALIGN_TOP_MID);
//     // lv_label_set_text(ui_Label1, "LVGL TEST label");

//     // ui_Switch1 = lv_switch_create(ui_Screen1);
//     // lv_obj_set_width(ui_Switch1, 50);
//     // lv_obj_set_height(ui_Switch1, 25);
//     // lv_obj_set_align(ui_Switch1, LV_ALIGN_BOTTOM_MID);

// 	// ui____initial_actions0 = lv_obj_create(NULL);
//     // lv_disp_load_scr(ui_Screen1);

// 	ui_init();

// 	// lv_obj_t* label = lv_label_create(lv_scr_act());
//     // lv_label_set_text(label, "LVGL TEST");
//     // lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);

// 	main_loop();

// 	// while (1) {
//     //     lv_timer_handler(); // let the GUI do its work
//     //     sleep_ms(10);
//     // }


//  	// OLED_ShowChar(0, 0, 'A', OLED_8X16);
//  	// OLED_ShowString(16, 0, "Hello World!", OLED_8X16);
//  	// OLED_ShowChar(0, 18, 'A', OLED_6X8);
//  	// OLED_ShowString(16, 18, "Hello World!", OLED_6X8);
//  	// OLED_ShowNum(0, 28, 12345, 5, OLED_6X8);
//  	// OLED_ShowSignedNum(40, 28, -66, 2, OLED_6X8);
//  	// OLED_Printf(0, 40, OLED_8X16, "%d.%d.%d-%s", lv_version_major(),lv_version_minor(), lv_version_patch(), lv_version_info());
//  	// OLED_Update();
//  	while (1);
//  }















#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"

// ST7735S commands
#define ST7735S_NOP     0x00
#define ST7735S_SWRESET 0x01
#define ST7735S_SLPOUT  0x11
#define ST7735S_DISPON  0x29
#define ST7735S_CASET   0x2A
#define ST7735S_RASET   0x2B
#define ST7735S_RAMWR   0x2C
#define ST7735S_COLMOD  0x3A

#define SPI_PORT spi0

#define PIN_SCK  2
#define PIN_MOSI 3
#define PIN_RST  4
#define PIN_DC   5
#define PIN_CS   6



void st7735s_write_command(uint8_t cmd) {
    gpio_put(PIN_DC, 0);
    gpio_put(PIN_CS, 0);
    spi_write_blocking(SPI_PORT, &cmd, 1);
    gpio_put(PIN_CS, 1);
}

void st7735s_write_data(uint8_t *data, size_t len) {
    gpio_put(PIN_DC, 1);
    gpio_put(PIN_CS, 0);
    spi_write_blocking(SPI_PORT, data, len);
    gpio_put(PIN_CS, 1);
}

void st7735s_init() {
    gpio_put(PIN_RST, 0);
    sleep_ms(100);
    gpio_put(PIN_RST, 1);
    sleep_ms(100);

    st7735s_write_command(ST7735S_SWRESET);
    sleep_ms(150);
    st7735s_write_command(ST7735S_SLPOUT);
    sleep_ms(500);

    uint8_t colmod_data = 0x05; // 16-bit color
    st7735s_write_command(ST7735S_COLMOD);
    st7735s_write_data(&colmod_data, 1);

    st7735s_write_command(ST7735S_DISPON);
    sleep_ms(100);
}

void st7735s_set_address_window(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1) {
    uint8_t data[4];
    st7735s_write_command(ST7735S_CASET);
    data[0] = 0x00;
    data[1] = x0;
    data[2] = 0x00;
    data[3] = x1;
    st7735s_write_data(data, 4);

    st7735s_write_command(ST7735S_RASET);
    data[0] = 0x00;
    data[1] = y0;
    data[2] = 0x00;
    data[3] = y1;
    st7735s_write_data(data, 4);
}

void st7735s_fill_color(uint16_t color) {
    st7735s_set_address_window(0, 0, 129, 160);
    st7735s_write_command(ST7735S_RAMWR);

    uint8_t data[2] = {color >> 8, color & 0xFF};
    gpio_put(PIN_DC, 1);
    gpio_put(PIN_CS, 0);
    for (int i = 0; i < 130 * 161; i++) {
        spi_write_blocking(SPI_PORT, data, 2);
    }
    gpio_put(PIN_CS, 1);
}

void st7735s_fill_color_(uint16_t color) {
    st7735s_set_address_window(0, 0, 129/2, 160);
    st7735s_write_command(ST7735S_RAMWR);

    uint8_t data[2] = {color >> 8, color & 0xFF};
    gpio_put(PIN_DC, 1);
    gpio_put(PIN_CS, 0);
    for (int i = 0; i < (130/2) * (161); i++) {
        spi_write_blocking(SPI_PORT, data, 2);
    }
    gpio_put(PIN_CS, 1);
}

int main() {
    stdio_init_all();
    spi_init(SPI_PORT, 40000000);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);
    gpio_init(PIN_DC);
    gpio_set_dir(PIN_DC, GPIO_OUT);
    gpio_init(PIN_RST);
    gpio_set_dir(PIN_RST, GPIO_OUT);

    st7735s_init();
    #define COLOR_RED   0xF800
    #define COLOR_GREEN 0x07E0
    #define COLOR_BLUE  0x001F

    while (1) {                                           
        st7735s_fill_color(COLOR_RED); // Red color
        sleep_ms(1000);
        st7735s_fill_color(COLOR_GREEN); // Red color
        sleep_ms(1000);
        st7735s_fill_color(COLOR_BLUE); // Red color
        sleep_ms(1000);
        st7735s_fill_color_(COLOR_RED);

        sleep_ms(1000);
        st7735s_fill_color_(COLOR_GREEN);

        sleep_ms(1000);
        st7735s_fill_color_(COLOR_BLUE);
    }

    return 0;
}


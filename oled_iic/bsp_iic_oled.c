#include "codetab.h"

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define I2C_PORT i2c0
#define OLED_ADDR 0x3C

// 初始化I2C接口
void Bsp_IIC_Init(void) {
    i2c_init(I2C_PORT, 100 * 1000); // 初始化I2C接口，速率为100kHz
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C); // 配置SDA引脚功能
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C); // 配置SCL引脚功能
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN); // 启用SDA引脚上拉
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN); // 启用SCL引脚上拉
}

// 向OLED寄存器地址写入一个字节的数据
void I2C_WriteByte(uint8_t addr, uint8_t data) {
    uint8_t buf[2] = {addr, data};
    i2c_write_blocking(I2C_PORT, OLED_ADDR, buf, 2, false);
}

// 向OLED写入命令
void WriteCmd(uint8_t cmd) {
    I2C_WriteByte(0x00, cmd);
}

// 向OLED写入数据
void WriteDat(uint8_t data) {
    I2C_WriteByte(0x40, data);
}

// 初始化OLED
void OLED_Init(void) {
    sleep_ms(100); // 延时

    WriteCmd(0xAE); // display off
    WriteCmd(0x20); // Set Memory Addressing Mode
    WriteCmd(0x10); // 00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
    WriteCmd(0xb0); // Set Page Start Address for Page Addressing Mode,0-7
    WriteCmd(0xc8); // Set COM Output Scan Direction
    WriteCmd(0x00); // ---set low column address
    WriteCmd(0x10); // ---set high column address
    WriteCmd(0x40); // --set start line address
    WriteCmd(0x81); // --set contrast control register
    WriteCmd(0xff); // 亮度调节 0x00~0xff
    WriteCmd(0xa1); // --set segment re-map 0 to 127
    WriteCmd(0xa6); // --set normal display
    WriteCmd(0xa8); // --set multiplex ratio(1 to 64)
    WriteCmd(0x3F); //
    WriteCmd(0xa4); // 0xa4,Output follows RAM content;0xa5,Output ignores RAM content
    WriteCmd(0xd3); // -set display offset
    WriteCmd(0x00); // -not offset
    WriteCmd(0xd5); // --set display clock divide ratio/oscillator frequency
    WriteCmd(0xf0); // --set divide ratio
    WriteCmd(0xd9); // --set pre-charge period
    WriteCmd(0x22); //
    WriteCmd(0xda); // --set com pins hardware configuration
    WriteCmd(0x12);
    WriteCmd(0xdb); // --set vcomh
    WriteCmd(0x20); // 0x20,0.77xVcc
    WriteCmd(0x8d); // --set DC-DC enable
    WriteCmd(0x14); //
    WriteCmd(0xaf); // --turn on oled panel
}

// 设置光标
void OLED_SetPos(uint8_t x, uint8_t y) {
    WriteCmd(0xb0 + y);
    WriteCmd(((x & 0xf0) >> 4) | 0x10);
    WriteCmd((x & 0x0f) | 0x01);
}

// 填充整个屏幕
void OLED_Fill(uint8_t fill_Data) {
    for (uint8_t m = 0; m < 8; m++) {
        WriteCmd(0xb0 + m); // page0-page1
        WriteCmd(0x00); // low column start address
        WriteCmd(0x10); // high column start address
        for (uint8_t n = 0; n < 128; n++) {
            WriteDat(fill_Data);
        }
    }
}

// 清屏
void OLED_CLS(void) {
    OLED_Fill(0x00);
}

// 将OLED从休眠中唤醒
void OLED_ON(void) {
    WriteCmd(0X8D); // 设置电荷泵
    WriteCmd(0X14); // 开启电荷泵
    WriteCmd(0XAF); // OLED唤醒
}

// 让OLED休眠 -- 休眠模式下,OLED功耗不到10uA
void OLED_OFF(void) {
    WriteCmd(0X8D); // 设置电荷泵
    WriteCmd(0X10); // 关闭电荷泵
    WriteCmd(0XAE); // OLED休眠
}

// 显示字符串，有6*8和8*16可选择
void OLED_ShowStr(uint8_t x, uint8_t y, uint8_t ch[], uint8_t TextSize) {
    uint8_t c = 0, i = 0, j = 0;
    switch (TextSize) {
        case 1: {
            while (ch[j] != '\0') {
                c = ch[j] - 32;
                if (x > 126) {
                    x = 0;
                    y++;
                }
                OLED_SetPos(x, y);
                for (i = 0; i < 6; i++)
                    WriteDat(F6x8[c][i]);
                x += 6;
                j++;
            }
        } break;
        case 2: {
            while (ch[j] != '\0') {
                c = ch[j] - 32;
                if (x > 120) {
                    x = 0;
                    y++;
                }
                OLED_SetPos(x, y);
                for (i = 0; i < 8; i++)
                    WriteDat(F8X16[c * 16 + i]);
                OLED_SetPos(x, y + 1);
                for (i = 0; i < 8; i++)
                    WriteDat(F8X16[c * 16 + i + 8]);
                x += 8;
                j++;
            }
        } break;
    }
}

// 显示汉字, 16*16点阵
void OLED_ShowCN(uint8_t x, uint8_t y, uint8_t N) {
    uint8_t wm = 0;
    uint16_t adder = 32 * N;
    OLED_SetPos(x, y);
    for (wm = 0; wm < 16; wm++) {
        WriteDat(F16x16[adder]);
        adder += 1;
    }
    OLED_SetPos(x, y + 1);
    for (wm = 0; wm < 16; wm++) {
        WriteDat(F16x16[adder]);
        adder += 1;
    }
}

// 显示BMP位图
void OLED_DrawBMP(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t BMP[]) {
    uint16_t j = 0;
    uint8_t x, y;

    if (y1 % 8 == 0)
        y = y1 / 8;
    else
        y = y1 / 8 + 1;
    for (y = y0; y < y1; y++) {
        OLED_SetPos(x0, y);
        for (x = x0; x < x1; x++) {
            WriteDat(BMP[j++]);
        }
    }
}

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

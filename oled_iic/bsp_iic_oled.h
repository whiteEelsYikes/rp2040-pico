#ifndef __BSP_IIC_OLED_H
#define __BSP_IIC_OLED_H

/* ��������ͷ�ļ� */
// #include "py32f0xx_hal.h"

 
/* �궨�� */
#define I2C_ADDRESS      0x78               /* Local address 0xA0 */
#define I2C_SPEEDCLOCK   400000             /* Communication speed 100K */
#define I2C_DUTYCYCLE    I2C_DUTYCYCLE_16_9 /* Duty cycle */



/* ȫ�ֱ������� */


/* �������� */
void Bsp_IIC_Init(void);

void OLED_Init(void);

void OLED_Fill(unsigned char fill_Data);//ȫ�����
    
void OLED_CLS(void);//����

void OLED_ON(void);

void OLED_OFF(void);

void OLED_ShowStr(unsigned char x, unsigned char y, unsigned char ch[], unsigned char TextSize);

void OLED_ShowCN(unsigned char x, unsigned char y, unsigned char N);

void OLED_ShowString(uint8_t Line, uint8_t Column, char *String);

void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);

void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length);

void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);

void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);

void OLED_DrawBMP(unsigned char x0,unsigned char y0,unsigned char x1,unsigned char y1,unsigned char BMP[]);



#endif /* __BSP_IIC_OLED_H */


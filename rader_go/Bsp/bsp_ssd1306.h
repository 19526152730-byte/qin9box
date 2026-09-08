#ifndef __BSP_SSD1306_H
#define __BSP_SSD1306_H

#include "stm32f4xx_hal.h"

/* 0.96" 128x64 OLED（SSD1306 控制器，I2C1，7 位地址 0x3C）简易驱动
 * 用法（在任务里调用，别在调度器启动前）：
 *   SSD1306_Init();                          // 屏没接时 ok=0，后续调用自动跳过，不会卡死
 *   SSD1306_Fill(0x00);
 *   SSD1306_DrawString(0, 0, "Person:", 1);  // 6x8 字体，scale=2 放大一倍
 *   SSD1306_DrawString(0, 16, "620 mm", 2);
 *   SSD1306_Flush();                         // 把显存推上屏
 */
uint8_t SSD1306_IsOK(void);
void    SSD1306_Init(void);
void    SSD1306_Fill(uint8_t pattern);
void    SSD1306_DrawChar(uint8_t x, uint8_t y, char c, uint8_t scale);
void    SSD1306_DrawString(uint8_t x, uint8_t y, const char *s, uint8_t scale);
void    SSD1306_Flush(void);

#endif /* __BSP_SSD1306_H */

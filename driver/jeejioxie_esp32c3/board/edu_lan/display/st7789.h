#ifndef __ST7789_H__
#define __ST7789_H__

#define USE_HORIZONTAL 0

#if USE_HORIZONTAL==0||USE_HORIZONTAL==1
#define LCD_W 240
#define LCD_H 320

#else
#define LCD_W 320
#define LCD_H 240
#endif

int vSt7789Init(void);
void vLcdSetWindow(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void vLcdClear(uint16_t color);
void vLcdDrawPoint(uint16_t x, uint16_t y);
uint16_t lLcdGetPointColor(void);
void vLcdSetPointColor(uint16_t color);
void vWriteData(uint8_t *data, int len);





#endif

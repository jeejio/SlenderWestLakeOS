/*
 * Copyright (c) 2022, jeejio Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author         Notes
 * 2023-02-17     xuyuhu         the first version
 */

#ifndef __TAL_DISPLAY_H__
#define __TAL_DISPLAY_H__

#include "string.h"

#define WHITE 0xFFFF
#define BLACK 0x0000
#define BLUE 0x001F
#define BRED 0XF81F
#define GRED 0XFFE0
#define GBLUE 0X07FF
#define RED 0xF800
#define MAGENTA 0xF81F
#define GREEN 0x07E0
#define CYAN 0x7FFF
#define YELLOW 0xFFE0
#define BROWN 0XBC40 // 棕色
#define BRRED 0XFC07 // 棕红色
#define GRAY 0X8430  // 灰色

#define DARKBLUE 0X01CF  // 深蓝色
#define LIGHTBLUE 0X7D7C // 浅蓝色
#define GRAYBLUE 0X5458  // 灰蓝色

#define LIGHTGREEN 0X841F // 浅绿色
#define LGRAY 0XC618      // 浅灰色

#define LGRAYBLUE 0XA651 // 浅灰蓝色
#define LBBLUE 0X2B12    // 浅棕蓝色

typedef enum
{
    LCD_TEXT_SIZE_12 = 12,
    LCD_TEXT_SIZE_16 = 16,
    LCD_TEXT_SIZE_24 = 24,
    LCD_TEXT_SIZE_32 = 32,
} lcd_text_size_t;

typedef struct
{
    uint8_t text_size;
    uint8_t f_red;
    uint8_t f_green;
    uint8_t f_blue;

    uint16_t point_color;

    uint8_t b_red;
    uint8_t b_green;
    uint8_t b_blue;
    uint16_t back_color;

    uint16_t start_x;
    uint16_t start_y;
    float word_space;
    float line_space;
} lcd_t;
extern lcd_t lcd;
/**
 * @brief       LCD初始化
 *      NOTE:   初始化LCD，并且将屏幕清屏为黑色
 *              默认文字前景色为白色，默认1倍字
 *              间距，默认1倍行间距
 *
 * @return 无
 */
int lTalDisplayInit(void);

/**
 * @brief       LCD清屏
 *      NOTE:   将LCD屏幕清空并设置为对应的颜色
 *
 * @param[in]   color  清屏对应的颜色值
 * @return 无
 */
void vTalDisplayClear(uint16_t color);

/**
 * @brief       设置文本颜色
 *      NOTE:   设置LCD屏幕的文字颜色
 * @param[in]   red    文字颜色的红色分量
 * @param[in]   green  文字颜色的绿色分量
 * @param[in]   blue   文字颜色的蓝色分量
 * @return 无
 */
void vTalDisplaySetTextColor(uint8_t red, uint8_t green, uint8_t blue);

/**
 * @brief       描点
 *      NOTE:   在LCD指定位置描点，位置通过x,y指定
 * @param[in]   x       描点的x坐标
 * @param[in]   y       描点的y坐标
 * @return 无
 */
void vTalDisplayDrawPoint(uint16_t x, uint16_t y);

/**
 * @brief       显示字符
 *      NOTE:   在LCD指定位置显示对应ASCII字符
 * @param[in]   x       字符的x坐标
 * @param[in]   y       字符的y坐标
 * @param[in]   chr     字符的ASCII码
 * @param[in]   size    ASCII字符的大小，支持12,16,24,32
 * @return 无
 */
void vTalDisplayShowChar(uint16_t x, uint16_t y, uint8_t chr, uint8_t size);

/**
 * @brief       设置LCD的显示区域
 *      NOTE:   设置显示区域后，可提高刷新显示速度
 * @param[in]   x1       矩形区域左上角x坐标
 * @param[in]   y1       矩形区域左上角y坐标
 * @param[in]   x2       矩形区域右下角x坐标
 * @param[in]   y2       矩形区域右下角y坐标
 * @return 无
 */
void vTalDisplaySetWindow(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);

/**
 * @brief	显示字符串
 *
 * @param   x,y		起点坐标
 * @param   width	字符显示区域宽度
 * @param   height	字符显示区域高度
 * @param   size	字体大小
 * @param   p		字符串指针
 *
 * @return  void
 */
void vTalDisplayShowString(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint8_t size, char *p);

/**
 * @brief	显示数字,高位为0的非有效位不显示
 *
 * @param   x,y		起点坐标
 * @param   num		需要显示的数字,数字范围(0~4294967295)
 * @param   len		需要显示的位数
 * @param   size	字体大小
 *
 * @return  void
 */
void vTalDisplayShowNum(uint16_t x, uint16_t y, unsigned long num, uint8_t len, uint8_t size);

/**
 * @brief	设置文本的颜色
 *
 * @param   color		RGB565格式的颜色值
 *
 * @return  void
 */
void vTalDisplaySetTextColor1(uint16_t color);

/**
 * @brief	获取文本大小
 *
 * @return  文字大小
 */
lcd_text_size_t xTalDisplayGetTextSize(void);

/**
 * @brief	设置文本大小
 *
 * @param   size	文字大小，支持12,16,24,32四种字体大小
 *
 * @return  无
 */
void vTalDisplaySetTextSize(lcd_text_size_t size);

/**
 * @brief	设置字间距
 *
 * @param   value	字间距，默认1.0倍字间距
 *
 * @return  无
 */
void vTalDisplaySetWordSpacing(float value);

/**
 * @brief	获取字间距
 *
 * @return  字间距
 */
float fTalDisplayGetWordSpacing(void);

/**
 * @brief	设置行间距
 *
 * @param   value	行间距，默认1.0倍行间距
 *
 * @return  无
 */
void vTalDisplaySetLineSpacing(float value);

/**
 * @brief	获取行间距
 *
 * @return  行间距
 */
float fTalDisplayGetLineSpacing(void);

/**
 * @brief	设置文本显示起始坐标
 *
 * @param   x,y		坐标值
 *
 * @return  无
 */
void vTalDisplaySetTextPosition(uint16_t x, uint16_t y);


#endif
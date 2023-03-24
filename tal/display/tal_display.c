/*
 * Copyright (c) 2022, jeejio Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author         Notes
 * 2023-02-17     xuyuhu         the first version
 */

#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tal_display.h"
#include "freertos/device.h"
#include "hal_sensor.h"
#include "drv_sensor_st7789.h"
#include "font.h"

#define DISPLAY_DEV_NAME "lcd_st7789"

lcd_t lcd;
jee_device_t screen_sensor_dev = NULL;
#define SCREEN_SET_IMAGE JEE_SENSOR_CTRL_USER_CMD_START + 1
#define SCREEN_SET_TEXT JEE_SENSOR_CTRL_USER_CMD_START + 2
#define SCREEN_SET_TEXT_COLOR JEE_SENSOR_CTRL_USER_CMD_START + 3
#define SCREEN_SET_TEXT_SIZE JEE_SENSOR_CTRL_USER_CMD_START + 4
#define SCREEN_SET_WORD_SPACING JEE_SENSOR_CTRL_USER_CMD_START + 5
#define SCREEN_SET_LINE_SPACING JEE_SENSOR_CTRL_USER_CMD_START + 6
#define SCREEN_SET_GIF JEE_SENSOR_CTRL_USER_CMD_START + 7
#define SCREEN_SET_START_POSITION JEE_SENSOR_CTRL_USER_CMD_START + 8

#define SCREEN_GET_TEXT_SIZE JEE_SENSOR_CTRL_USER_CMD_START + 9
#define SCREEN_GET_WORD_SPACING JEE_SENSOR_CTRL_USER_CMD_START + 10
#define SCREEN_GET_LINE_SPACING JEE_SENSOR_CTRL_USER_CMD_START + 11

static int buffer[128];
lcd_t lcd;

void vTalDisplayWriteBytes(uint8_t *buff, uint16_t len)
{
    lcd_data_t data = {len, buff};
    jee_device_control(screen_sensor_dev, SENSOR_DISPLAY_WRITE_DATA, &data);
}

void vTalDisplayShowChar(uint16_t x, uint16_t y, uint8_t chr, uint8_t size)
{
    uint8_t temp, t1, t;
    uint8_t csize; // 得到字体一个字符对应点阵集所占的字节数
    uint16_t colortemp;
    uint8_t sta;
    uint8_t data_buff[2];
    uint8_t pos = chr - ' '; // 得到偏移后的值（ASCII字库是从空格开始取模，所以-' '就是对应字符的字库）
    if ((x > (LCD_W - size / 2)) || (y > (LCD_H - size)))
    {
        return;
    }
    vTalDisplaySetWindow(x, y, x + size / 2 - 1, y + size - 1); //(x,y,x+8-1,y+16-1)
    if ((size == 16) || (size == 32))                           // 16和32号字体
    {
        csize = (size / 8 + ((size % 8) ? 1 : 0)) * (size / 2);
        for (t = 0; t < csize; t++)
        {
            if (size == 16)
            {
                temp = asc2_1608[pos][t]; // 调用1608字体
            }
            else if (size == 32)
            {
                temp = asc2_3216[pos][t]; // 调用3216字体
            }
            else
            {
                return; // 没有的字库
            }
            for (t1 = 0; t1 < 8; t1++)
            {
                if (temp & 0x80)
                {
                    colortemp = lcd.point_color;
                }
                else
                {
                    colortemp = lcd.back_color;
                }

                data_buff[0] = colortemp >> 8;
                data_buff[1] = colortemp;
                vTalDisplayWriteBytes(data_buff, 2);
                temp <<= 1;
            }
        }
    }
    else if (size == 12) // 12号字体
    {
        csize = (size / 8 + ((size % 8) ? 1 : 0)) * (size / 2);
        for (t = 0; t < csize; t++)
        {
            temp = asc2_1206[pos][t];
            for (t1 = 0; t1 < 6; t1++)
            {
                if (temp & 0x80)
                {
                    colortemp = lcd.point_color;
                }
                else
                {
                    colortemp = lcd.back_color;
                }
                data_buff[0] = colortemp >> 8;
                data_buff[1] = colortemp;
                vTalDisplayWriteBytes(data_buff, 2);
                temp <<= 1;
            }
        }
    }
    else if (size == 24) // 24号字体
    {
        csize = (size * 16) / 8;
        for (t = 0; t < csize; t++)
        {
            temp = asc2_2412[pos][t];
            if (t % 2 == 0)
            {
                sta = 8;
            }
            else
            {
                sta = 4;
            }
            for (t1 = 0; t1 < sta; t1++)
            {
                if (temp & 0x80)
                {
                    colortemp = lcd.point_color;
                }
                else
                {
                    colortemp = lcd.back_color;
                }
                data_buff[0] = colortemp >> 8;
                data_buff[1] = colortemp;
                vTalDisplayWriteBytes(data_buff, 2);
                temp <<= 1;
            }
        }
    }
}

/**
 * @brief	m^n函数
 *
 * @param   m,n		输入参数
 *
 * @return  m^n次方
 */
static unsigned long ulLcdPower(uint8_t m, uint8_t n)
{
    unsigned long result = 1;
    while (n--)
    {
        result *= m;
    }
    return result;
}


void vTalDisplayShowNum(uint16_t x, uint16_t y, unsigned long num, uint8_t len, uint8_t size)
{
    uint8_t t, temp;
    uint8_t enshow = 0;
    for (t = 0; t < len; t++)
    {
        temp = (num / ulLcdPower(10, len - t - 1)) % 10;
        if (enshow == 0 && t < (len - 1))
        {
            if (temp == 0)
            {
                vTalDisplayShowChar(x + (size / 2) * t, y, ' ', size);
                continue;
            }
            else
            {
                enshow = 1;
            }
        }
        vTalDisplayShowChar(x + (size / 2) * t, y, temp + '0', size);
    }
}

#define IMG_PACK_SIZE (1024)
void vTalDisplayShowImage(int x1, int y1, int x2, int y2, uint8_t *img)
{
    uint32_t img_len = (x2 - x1 + 1) * (y2 - y1 + 1);
    uint32_t remain = img_len; //像素点为单位
    uint32_t packet_size = LCD_W * 64;
    int pix_pos = 0;
    vTalDisplaySetWindow(x1, y1, x2, y2);
    while (remain > 0)
    {
        if (remain >= IMG_PACK_SIZE)
        {
            vTalDisplayWriteBytes(&img[pix_pos], IMG_PACK_SIZE * 2);
            pix_pos += IMG_PACK_SIZE * 2;
            remain -= IMG_PACK_SIZE;
        }
        else
        {
            vTalDisplayWriteBytes(&img[pix_pos], remain * 2);
            remain = 0;
        }
    }
}

/**
 * @brief	显示数字,高位为0,可以控制显示为0还是不显示
 *
 * @param   x,y		起点坐标
 * @param   num		需要显示的数字,数字范围(0~999999999)
 * @param   len		需要显示的位数
 * @param   size	字体大小
 * @param   mode	1:高位显示0		0:高位不显示
 *
 * @return  void
 */
void vTalDisplayShowXNum(uint16_t x, uint16_t y, unsigned long num, uint8_t len, uint8_t size, uint8_t mode)
{
    uint8_t t, temp;
    uint8_t enshow = 0;
    for (t = 0; t < len; t++)
    {
        temp = (num / ulLcdPower(10, len - t - 1)) % 10;
        if (enshow == 0 && t < (len - 1))
        {
            if (temp == 0)
            {
                if (mode)
                {
                    vTalDisplayShowChar(x + (size / 2) * t, y, '0', size);
                }
                else
                {
                    vTalDisplayShowChar(x + (size / 2) * t, y, ' ', size);
                }
                continue;
            }
            else
            {
                enshow = 1;
            }
        }
        vTalDisplayShowChar(x + (size / 2) * t, y, temp + '0', size);
    }
}

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
void vTalDisplayShowString(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint8_t size, char *p)
{
    uint8_t x0 = x;
    // width += x;
    // height += y;
    uint8_t space = 0;
    while ((*p <= '~') && (*p >= ' ')) //判断是不是非法字符!
    {
        if (x >= width-size/2)
        {
            x = x0;
            y += size * lcd.line_space;
        }
        if (y >= height)
        {
            break; //退出
        }
        vTalDisplayShowChar(x, y, *p, size);
        x += (size / 2) * lcd.word_space;
        p++;
    }
    lcd.start_x = 0;
    lcd.start_y = y + size * lcd.line_space;
}

void vTalDisplaySetWindow(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    uint16_t buff[4] = {x1, y1, x2, y2};

    jee_device_control(screen_sensor_dev, SENSOR_DISPLAY_SET_WINDOW, buff);
}

void vTalDisplayDrawPoint(uint16_t x, uint16_t y)
{
    uint16_t pos[2] = {x, y};
    jee_device_control(screen_sensor_dev, SENSOR_DISPLAY_DRAW_POINT, pos);
}

void vTalDisplaySetTextColor(uint8_t red, uint8_t green, uint8_t blue)
{
    lcd.f_red = red;
    lcd.f_blue = blue;
    lcd.f_green = green;
    lcd.point_color = (((uint16_t)red & 0xf8) << 8) +
                      (((uint16_t)green & 0xFC) << 3) +
                      (((uint16_t)blue & 0xF8) >> 3);
    jee_device_control(screen_sensor_dev, SENSOR_DISPLAY_SET_POINT_COLOR, &lcd.point_color);
}

void vTalDisplaySetTextColor1(uint16_t color)
{
    lcd.point_color = color;
    jee_device_control(screen_sensor_dev, SENSOR_DISPLAY_SET_POINT_COLOR, &lcd.point_color);
}

void vTalDisplaySetTextSize(lcd_text_size_t size)
{
    lcd.text_size = size;
}

lcd_text_size_t xTalDisplayGetTextSize(void)
{
    return lcd.text_size;
}

void vTalDisplaySetWordSpacing(float value)
{
    lcd.word_space = value;
}

float fTalDisplayGetWordSpacing(void)
{
    return lcd.word_space;
}

void vTalDisplaySetLineSpacing(float value)
{
    lcd.line_space = value;
}

float fTalDisplayGetLineSpacing(void)
{
    return lcd.line_space;
}

void vTalDisplaySetTextPosition(uint16_t x, uint16_t y)
{
    lcd.start_x = x;
    lcd.start_y = y;
}

void vTalDisplayClear(uint16_t color)
{
    jee_device_control(screen_sensor_dev, SENSOR_DISPLAY_CLEAR, &color);
}

int lTalDisplayInit(void)
{
    // 查找设备
    screen_sensor_dev = jee_device_find(DISPLAY_DEV_NAME);
    if (screen_sensor_dev == JEE_NULL)
    {
        LOGI("display", "can not find sensor Model\n");
        return -1;
    }
    else
        LOGI("display", "find sensor Model ok\n");

    // 打开设备
    jee_err_t result = JEE_EOK;
    result = jee_device_open(screen_sensor_dev, JEE_DEVICE_FLAG_RDONLY);
    if (result != JEE_EOK)
    {
        LOGI("display", "can not open senor device\n");
        return -1;
    }
    else
        LOGI("display", "open senor device ok\n");
    vTalDisplaySetTextSize(LCD_TEXT_SIZE_16);
    vTalDisplaySetLineSpacing(1);
    vTalDisplaySetWordSpacing(1);
    vTalDisplayClear(BLACK);
    vTalDisplaySetTextColor1(CYAN);
    //vTalDisplaySetTextColor(255,0,0);
    vTalDisplayShowString(50,100,LCD_W,LCD_H,LCD_TEXT_SIZE_24,"Hello Jeejio!");
    // for test
    // extern const unsigned char gImage_Pic[128640];
    // vTalDisplayShowImage(0,0,201-1,320-1,gImage_Pic);

    return 0;
}

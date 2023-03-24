#ifndef __DRV_SENSOR_ST7789_H__
#define __DRV_SENSOR_ST7789_H__

#include "stdint.h"
#include "st7789.h"     //厂家驱动头文件
typedef struct
{
    uint16_t length;
    uint8_t *buffer;
} lcd_data_t;

typedef enum
{
    SENSOR_DISPLAY_INIT = (JEE_SENSOR_CTRL_USER_CMD_START + 1),
    SENSOR_DISPLAY_CLEAR,
    SENSOR_DISPLAY_SET_POINT_COLOR,
    SENSOR_DISPLAY_SET_WINDOW,
    SENSOR_DISPLAY_DRAW_POINT,
    SENSOR_DISPLAY_WRITE_DATA,
}sensor_cmd_t;

#endif

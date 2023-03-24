#ifndef __TAL_LED_ARRAY_H__
#define __TAL_LED_ARRAY_H__

#define WS2835_PIXEL_SET_CMD (JEE_SENSOR_CTRL_USER_CMD_START + 1)
#define WS2835_REFRESH_SET_CMD (JEE_SENSOR_CTRL_USER_CMD_START + 2)
#define WS2835_CLEAR_SET_CMD (JEE_SENSOR_CTRL_USER_CMD_START + 3)
#define WS2835_DEL_SET_CMD (JEE_SENSOR_CTRL_USER_CMD_START + 4)

typedef enum
{
    ledArryFlashModeFast,
    ledArryFlashModeMiddle,
    ledArryFlashModeSlow,
    ledArryFlashModeNull,

} FlashMode;

typedef struct
{
    jee_uint8_t red;
    jee_uint8_t green;
    jee_uint8_t blue;

} color_t;

typedef struct
{
    jee_uint8_t power;
    jee_uint8_t bright;
    color_t color;
    color_t colorTarget;
    jee_uint8_t flashOnoff;
    jee_uint8_t flashCount;
} led_t;


 /**
   * @brief       led矩阵初始化
   *
   * NOTE:        TAL层调用驱动框架查找设备，并对led矩阵硬件进行初始化
   *
   */
jee_int32_t lTalLedArrayInit(void);

 /**
   * @brief       设置led矩阵开关
   *
   * @param[in]  onOff ： 1-开，0-关
   *
   */
void vTalLedArraySetOnOff(jee_uint8_t onOff);

 /**
   * @brief       获取led矩阵开关状态
   *
   * @return      1-开，0-关
   *
   */
jee_uint8_t ucTalLedArrayGetStatus(void);

 /**
   * @brief       设置led矩阵闪烁次数
   *
   * @param[in]   flashNumber  :闪烁次数
   *
   */
void vTalLedArraySetFlashNumber(jee_uint32_t flashNumber);

 /**
   * @brief       设置led矩阵闪烁速度
   *
   * @param[in]   speed  :闪烁速度
   *
   */
void vTalLedArraySetFlashMode(FlashMode speed);

 /**
   * @brief       获取led矩阵闪烁速度
   *
   * @return      闪烁速度类型（FlashMode）
   * 
   */
FlashMode xTalLedArrayGetFlashMode(void);

 /**
   * @brief       设置led矩阵颜色
   *
   * @param[in]   color 颜色结构体（color_t）
   *
   */
void vTalLedArraySetRgbColor(color_t color);

 /**
   * @brief       获取led矩阵闪烁速度
   *
   * @return      颜色结构体（color_t）
   * 
   */
color_t xTalLedArrayGetRgbColor(void);

#endif

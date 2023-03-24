#ifndef __RAIN_SENSOR_H__
#define __RAIN_SENSOR_H__

#include "string.h"
#include "stdint.h"
#include "stdbool.h"

typedef enum
{
    None,
    Light=1,
    Middle,
    Heavy
} hal_rain_level_t;


/**
 * @brief       雨水传感器初始化
 * 
 * @return      无
 */
 void hyetometer_init(void);

/**
 * @brief       雨水报警状态查询
 * 
 * @return      报警状态 true：触发报警  false：未触发报警
 */
 bool hyetometer_get_status(void);

/**
 * @brief       雨量查询
 * 
 * @return      雨量等级值 None：未下雨  Light:小雨  Middle：中雨  Heavy：大雨
 */
 uint8_t hyetometer_get_level(void);

 /**
 * @brief       雨量数据打包
 * 
 * @param[out]  str     打包后数据指针
 * 
 * @return      雨量等级值 None：未下雨  Light:小雨  Middle：中雨  Heavy：大雨
 */
uint8_t rian_sensor_data_pack(char *str);

void hyetometer_get_info(int *buf);

#endif

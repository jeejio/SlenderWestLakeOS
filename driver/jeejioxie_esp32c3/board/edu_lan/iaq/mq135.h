#ifndef __MQ135_H
#define __MQ135_H

#include "jeedef.h"

/**
 * @brief       空气质量传感器初始化
 *
 * NOTE:        在其他 空气质量传感器 相关函数使用前调用，只调用一次
 *
 */
jee_int32_t mq135_init(void);

/**
 * @brief       获取空气质量超标状态
 *
 * @return      空气质量超标状态值，
 *              0：未超标；
 *              1：超标。
 *
 */
jee_int8_t mq135_get_data(void);
#endif

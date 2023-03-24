#ifndef __MQ7_H
#define __MQ7_H

#include "jeedef.h"

/**
 * @brief       一氧化碳传感器初始化
 *
 * NOTE:        在其他 一氧化碳传感器 相关函数使用前调用，只调用一次
 *
 */
jee_int32_t mq7_init(void);

/**
 * @brief       获取一氧化碳传感器超标值
 *
 * @return      一氧化碳超标状态值，
 *              0：未超标；
 *              1：超标。
 *
 */
jee_int8_t mq7_get_data(void);

#endif

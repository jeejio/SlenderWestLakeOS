#ifndef __MQ2_H
#define __MQ2_H

#include "stdint.h"
#include "stdbool.h"
#include "jeedef.h"

/**
 * @brief       烟雾传感器初始化
 *
 * NOTE:        在其他 烟雾传感器 相关函数使用前调用，只调用一次
 *
 */
jee_int32_t mq2_init(void);

/**
 * @brief       获取烟雾超标状态
 *
 * @return      烟雾超标状态值
 *              0：未超标；
 *              1：超标。
 *
 */
jee_int8_t mq2_get_data(void);

#endif

#ifndef __CC6201ST_H
#define __CC6201ST_H

#include "stdint.h"
#include "jeedef.h"

#define MAG_ID1 (1)
#define MAG_ID2 (2)
#define MAG_ID3 (3)

/**
 * @brief             霍尔传感器初始化
 * 
 *   NOTE:            初始化3路霍尔传感器
 */
void cc6201stInit(void);

/**
 * @brief             霍尔传感器初始化
 * 
 * @param[in]  id     霍尔传感器通道号，取值范围：1，2，3
 * 
 * @return            返回相应通道的磁场传感器状态，返回低电平表示检测到磁场
 */
jee_int32_t cc6201stRead(jee_uint8_t id);


/**
 * @brief             霍尔传感器状态获取
 * 
 *   NOTE:            读取到低电平表示检测到磁场，返回值对应Bit为1表示检测到磁场
 * 
 * @return            三路霍尔传感器的状态值，低3 Bit的值分别表示三路传感器的状态
 */
jee_uint8_t cc6201stGetStatus(void);

#endif
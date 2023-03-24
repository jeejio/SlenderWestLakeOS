#ifndef __LB1427B_H
#define __LB1427B_H

#include "stdint.h"
#include "jeedef.h"

/**
   * @brief       初始化水泵设备.
   *
   *   NOTE:      PWM初始化
   */
void lb1427Init(void);

/**
   * @brief       获取水泵PWM占空比.
   *
   * @return      返回 PWM占空比 0-100
   * 
   */

jee_uint8_t lb1427GetPwmDuty(void);

/**
   * @brief       设置水泵PWM占空比.
   *
   * @param[in]   value - PWM占空比 0-100
   * 
   */
void lb1427SetPwmDuty(jee_uint8_t value);

#endif

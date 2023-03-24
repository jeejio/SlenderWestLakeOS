#ifndef __SBF01_H__
#define __SBF01_H__

#include "jeedef.h"

#define PIR_PIN_NUM (2)


 /**
   * @brief       SBF_01硬件初始化
   *
   * NOTE:        -
   *
   */
void vSbf01Init(void);

 /**
   * @brief       读取SBF_01的数据
   *
   * NOTE:        -
   * 
   * @return      1:检测到活体 0：未检测到活体
   *
   */
jee_base_t lSbf01ReadStatus(void);

#endif

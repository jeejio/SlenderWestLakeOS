#ifndef __BH1750FVI_H
#define __BH1750FVI_H

#include "stdint.h"
#include "jeedef.h"


 /**
   * @brief       环境光传感器初始化
   * 
   * NOTE:        在其他 环境光传感器 相关函数使用前调用，只调用一次 
   * 
  */
void bh1750fviInit(void);


/**
   * @brief       获取光照强度值
   * 
   * @return      光照强度值，单位：lx      
   * 
  */

uint16_t bh1750fviGetData(void);

#endif

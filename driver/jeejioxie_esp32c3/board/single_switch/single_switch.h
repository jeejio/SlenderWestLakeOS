#ifndef __SINGLE_SWITCH_H__
#define __SINGLE_SWITCH_H__

#include "stdint.h"



  /**
   * @brief       单键开关初始化 
   * 
   *   NOTE:      gpio 设置为输入模式
   * 
   */
void single_switch_init(void);

  /**
   * @brief       清楚模式记录
   *  
   */
void single_switch_clean_mode(void);

  /**
   * @brief       获取开关信息
   * 
   *    NOTE:     buf[0]:模式   buf[1]:是否按压
   *  
   */
void single_switch_get_info(int *buf);

#endif

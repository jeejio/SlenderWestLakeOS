#ifndef __MAX30102_H
#define __MAX30102_H

#include "stdint.h"


/**
   * @brief       初始化心率血氧设备.
   *
   *   NOTE:      PWM初始化
   */
void bloodoxygen_sensor_init(void);

/**
   * @brief       获取心率和血氧浓度数据.
   *
   * @param[out]  n_sp02 - 血氧浓度
   * 
   * @param[out]  n_heart_rate - 心率
   * 
   */
void bloodoxygen_sensor_get_data(int *n_sp02, int *n_heart_rate);

/**
   * @brief       获取检测仪的状态（开/关）.
   *
   * @return      检测仪的开关状态（开：1、关：0）   
   */
uint8_t bloodoxygen_get_onoff(void);

/**
   * @brief       设置检测仪的状态（开/关）.
   *
   * @param[in]  flag - 开关标志（开：1、关：0） 
   * 
   */
void bloodoxygen_set_onoff(uint8_t flag);

uint8_t bloodoxygen_get_send_flag(void);

#endif

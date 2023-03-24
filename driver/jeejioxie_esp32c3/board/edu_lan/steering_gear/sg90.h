#ifndef __SG90_H
#define __SG90_H

#include "stdint.h"

#define CLOCK_WISE (1)
#define COUNTER_CLOCK_WISE (0)

typedef struct
{
    int current_dirction;
    int set_dirction;
    int current_angle;
    int set_angle;
} steering_gear_config_t;

/**
   * @brief       初始化舵机设备.
   *
   *   NOTE:      PWM初始化
   */
void sg90Init(void);

/**
   * @brief       获取舵机角度.
   *
   *   NOTE:      角度范围0-180
   *
   * @return      舵机角度
   */
int steeringGearGetAngle(void);

/**
   * @brief       设置舵机旋转角度.
   *
   * @param[in]  angel - 旋转角度
   * 
   *   NOTE:      角度范围0-180   
   */
void steeringGearSetAngle(int angel);

/**
   * @brief       获取舵机旋转方向.
   *
   *   NOTE:      顺时针或逆时针方向
   * 
   * @return      舵机旋转方向（ 顺时针：1、逆时针：0）   
   */
int steeringGearGetDirction(void);

/**
   * @brief       设置舵机旋转方向.
   *
   *   NOTE:      顺时针或逆时针方向
   * 
   * @param[in]  valule - 旋转方向（ 顺时针：1、逆时针：0）   
   */
void steeringGearSetDirction(int valule);

/**
   * @brief      测试代码，舵机自动旋转.
   *
   */
void steeringGearTask(void *arg);


/**
   * @brief       获取舵机旋转方向和角度.
   *
   *   NOTE:      顺时针或逆时针方向，0-180°
   * 
   * @param[out]  pdata - 旋转方向（ 顺时针：1、逆时针：0）,角度   
   */
void steeringGearGetDirctionAngle(uint32_t *pdata);




#endif

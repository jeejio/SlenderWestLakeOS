/*
 * Copyright (c) 2022, jeejio Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author         Notes
 * 2023-02-01     xuyuhu         the first version
*/

#ifndef __TAL_STEERING_GEAR_H__
#define __TAL_STEERING_GEAR_H__

#include "string.h"

typedef struct
{
    int current_dirction;
    int set_dirction;
    int current_angle;
    int set_angle;
} SteeringGearConfig_t;

/**
   * @brief       初始化舵机设备.
   *
   *   NOTE:      PWM初始化
   *   
   * @return      0 :success    other :fail
   */
int vTalSteeringGearInit(void);

/**
   * @brief       获取舵机角度.
   *
   *   NOTE:      角度范围0-180
   *
   * @return      舵机角度
   */
int lTalSteeringGearGetRotationAngle(void);

/**
   * @brief       设置舵机旋转角度.
   *
   * @param[in]  angel - 旋转角度
   * 
   *   NOTE:      角度范围0-180   
   */
void vTalSteeringGearSetRotationAngle(int angel);

/**
   * @brief       获取舵机旋转方向.
   *
   *   NOTE:      顺时针或逆时针方向
   * 
   * @return      舵机旋转方向（ 顺时针：1、逆时针：0）   
   */
int lTalSteeringGearGetRotationDirction(void);

/**
   * @brief       设置舵机旋转方向.
   *
   *   NOTE:      顺时针或逆时针方向
   * 
   * @param[in]  valule - 旋转方向（ 顺时针：1、逆时针：0）   
   */
void vTalSteeringGearSetRotationDirction(int valule);

#endif
/*
 * Copyright (c) 2022, jeejio Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author         Notes
 * 2023-02-14     xuyuhu         the first version
 */

#ifndef __TAL_JOYSTICK_ENCODER_H__
#define __TAL_JOYSTICK_ENCODER_H__

#include "string.h"
typedef struct
{
    int getPositioninformationX;
    int getPositioninformationY;
    int IsMode;
    int IsPress;
} TalJoystickInfo_t;

typedef struct
{
    int getPositioninformationX;
    int getPositioninformationY;
} TalJoystickPositionInfo_t;

/**
 * @brief       游戏摇杆初始化
 *
 * @return      无
 */
int vTalJoystickInit(void);

/**
 * @brief       获取按压状态
 *
 * @return      0 松开  1 按下
 */
int lTalJoystickGetStatus(void);

/**
 * @brief       获取按键模式
 *
 * @return      1 单击 2 双击  3 长按
 */
int lTalJoystickModeReceive(void);

/**
 * @brief       获取X轴
 *
 * @return      数值
 */
int lTalJoystickGetPositioninformationX(void);

/**
 * @brief       获取Y轴
 *
 * @return      数值
 */
int lTalJoystickGetPositioninformationY(void);

/**
 * @brief       获取XY轴
 *
 * @return      数值
 */
TalJoystickPositionInfo_t *lTalJoystickPositionReceive(void);

#endif
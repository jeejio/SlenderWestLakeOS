/*
 * Copyright (c) 2022, jeejio Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author         Notes
 * 2023-03-23     xuyuhu         the first version
 */

#ifndef __TAL_SINGLE_SWITCH_H__
#define __TAL_SINGLE_SWITCH_H__

#include "string.h"
typedef struct
{
    int IsMode;
    int IsPress;
} TalSwitchInfo_t;


/**
 * @brief       单键开关初始化
 *
 * @return      无
 */
int vTalSingleSwitchInit(void);

/**
 * @brief       获取按压状态
 *
 * @return      0 松开  1 按下
 */
int lTalSingleSwitchGetStatus(void);

/**
 * @brief       获取开关模式
 *
 * @return      1 单击 2 双击  3 长按
 */
int lTalSingleSwitchModeReceive(void);

#endif
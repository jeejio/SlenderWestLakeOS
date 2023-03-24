/*
 * Copyright (c) 2022, jeejio Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author         Notes
 * 2023-02-10     xuyuhu         the first version
*/

#ifndef __TAL_HYETOMETER_ENCODER_H__
#define __TAL_HYETOMETER_ENCODER_H__

#include "string.h"

typedef struct
{
    int isRaining;
    int precipitationLevel;
} HyetomterInfo_t;

/**
 * @brief       雨水传感器初始化
 * 
 * @return      无
 */
int vTalHyetometerInit(void);

/*******以下接口与 【中科物栖小应用开发文档】 保持函数名称对应**********/

/**
 * @brief       雨水报警状态查询
 *
 * @return      报警状态 1：触发报警  0：未触发报警
 */
int cTalHyetometerGetIsRaining(void);

/**
 * @brief       雨量查询
 *
 * @return      雨量等级值 0：未下雨  1:小雨  2：中雨  3：大雨
 */
int cTalHyetometerGetPecipitationLevel(void);

 /**
 * @brief       雨量数据打包
 * 
 * 
 * @return         int isRaining;
 *                 int precipitationLevel;
 */
HyetomterInfo_t *xTalHyetometerOnReceive(void);
#endif
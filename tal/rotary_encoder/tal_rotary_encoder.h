/*
 * Copyright (c) 2022, jeejio Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author         Notes
 * 2023-02-03     xuyuhu         the first version
*/

#ifndef __TAL_ROTARY_ENCODER_H__
#define __TAL_ROTARY_ENCODER_H__

#include "string.h"

typedef struct
{
    int IsPressed;          /* 获取按钮的状态(是否按压) */
    int RotationDirection;  /* 获取旋转方向 */
    int RelativePos;        /* 获取编码器档位 */
} RotaryEncoderStatus_t;

/**
   * @brief       初始化选择编码器                .
   *
   *   NOTE:      PWM初始化
   */
int vTalRotaryEncoderInit(void);

/*******以下接口与 【中科物栖小应用开发文档】 保持函数名称对应**********/
/**
   * @brief       获取编码器档位和OK键状态和按钮状态.
   *
   *   NOTE:      编码器档位(0～20档，20个档位=360度） 
   *              按键状态（ 按下：1、抬起：0）   
   *              方向（ 顺时针：1、逆时针：0） 
   * @param[out]  pdata - 档位，按键状态，OK键状态  
   */
RotaryEncoderStatus_t * xTalRotaryEncoderOnReceive(void);

/**
   * @brief       获取编码器档位.
   *
   * @return      编码器档位(0～20档，20个档位=360度） 
   */
int lTalRotaryEncoderGetPosition(void);

/**
   * @brief       获取按钮的状态(是否按压).
   *
   * @return      按键状态（ 按下：1、抬起：0）   
   */
bool cTalRotaryEncoderGetIsPressed(void);

/**
   * @brief       获取编码器OK键.
   *
   * @return      方向（ 顺时针：1、逆时针：0）   
   */
bool cTalRotaryEncoderGetRotationDirection(void);

#endif
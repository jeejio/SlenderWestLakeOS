/*
 * Copyright (c) 2022, jeejio Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author         Notes
 * 2023-02-10     xuyuhu         the first version
*/

#ifndef __TAL_BUTTON_PANEL_H__
#define __TAL_BUTTON_PANEL_H__

#include "string.h"
#include "stdint.h"
typedef struct
{
    int code;
    int value;
    int button_mode;
} ButtonPanelInfo_t;

/**
 * @brief       矩阵按键初始化
 * 
 * @return      无
 */
int vTalButtonPanelInit(void);

/*******以下接口与 【中科物栖小应用开发文档】 保持函数名称对应**********/
 /**
   * @brief       获取按下的按键序号
   * 
   * NOTE:        获取最近一次完成按下操作的按键序号 
   * 
   * @return      0： 没有按键按下，1-9：按下的按键序号
   * 
   */
 int  cTalButtonPanelGetButtonCodes(void);

 /**
   * @brief       按键按下模式
   * 
   * NOTE:        获取最近一次完成按下操作的按键模式
   * 
   * @return     按键按下模式
   *             1 - 单击 @ref 500ms 秒内完成按键动作  
   *             2 - 双击 @ref 连续两次完成单击动作 
   *             3 - 多击 @ref 连续三次及以上完成单击动作
   *             5 - 短按 @ref 500ms-5s 完成单次按下动作
   *             7 - 长按 @ref 5s-9s 内完成单次按下动作
   *             9 - 长按保持 @ref 超过9s 完成单次按下动作
   */
int cTalButtonPanelGetButtonMode(void);

 /**
 * @brief       获取按键信息
 * 
 *   NOTE:      按键序号和模式
 *             序号（1-9）
 *             模式类型如下：
 *             1 - 单击 @ref 500ms 秒内完成按键动作  
 *             2 - 双击 @ref 连续两次完成单击动作 
 *             3 - 多击 @ref 连续三次及以上完成单击动作
 *             5 - 短按 @ref 500ms-5s 完成单次按下动作
 *             7 - 长按 @ref 5s-9s 内完成单次按下动作
 *             9 - 长按保持 @ref 超过9s 完成单次按下动作
 * 
 * @return     字符串，格式如：{\"code\":1,\"value\":%d,\"buttonMode\":%d}
 */
ButtonPanelInfo_t * cTalButtonPanelOnReceive(void);

#endif  

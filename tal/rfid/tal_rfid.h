/*
 * Copyright (c) 2022, jeejio Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author         Notes
 * 2023-02-08     xuyuhu         the first version
 */

#ifndef __TAL_RFID_H__
#define __TAL_RFID_H__

#include "string.h"

typedef struct
{
    uint8_t id[4];
    uint8_t data[16];
} RfidInfo_t;

/**
 * @brief       RFID初始化
 *
 *   NOTE:      配置GPIO，配置RFID参数
 */
void vTalRfidInit(void);
/*******以下接口与 【中科物栖小应用开发文档】 保持函数名称对应**********/
/**
 * @brief       设置卡的物理区块的信息内容.
 *
 *   NOTE:      dat 长度 16字节
 *
 * @param[in]  *dat - 输入信息.
 */
void vTalRfidSetData(uint8_t *data);

  /**
   * @brief       获取卡自身的id.
   *
   *   NOTE:      id 长度 4字节
   *
   * @param[out]  *id - 输出ID.
   * 
   */
void vTalRfidGetID(uint8_t *id);

  /**
   * @brief       获取卡的物理区块的信息内容.
   *
   *   NOTE:      id  长度 4字节
   *              dat 长度 16字节
   *
   * @param[out]  *id  - 输出ID.
   *
   * @param[out]  *dat - 输出信息. 
   */
void vTalRfidOnReceive(uint8_t *id,uint8_t *dat);

  /**
   * @brief       获取卡的物理区块的信息内容.
   *
   *   NOTE:     
   *              dat 长度 16字节
   *
   * @param[out]  *dat - 输出信息. 
   */
void vTalRfidGetData(uint8_t *dat);

#endif
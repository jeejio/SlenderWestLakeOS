/*
 * Copyright (c) 2022, jeejio Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author         Notes
 * 2023-02-22     xuyuhu         the first version
*/

#ifndef __FACTORY_TEST_H__
#define __FACTORY_TEST_H__

#include "stdint.h"

enum RESULT
{
    RE_SUCCESS,
    RE_FAILED
};
enum MCU_PACKAGE_TYPE
{
    CMD,
    KEY,
    REPORT,
    READ_SN_MAC
};
typedef struct
{
    uint8_t status[4]; // 0
    uint32_t length;   // 4
    char hascode[64];  // 8
    uint8_t data[512]; // 72
} MCU_KEY_PACKAGE;

void factory_test_init(void);


#endif
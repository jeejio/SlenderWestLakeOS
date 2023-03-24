/*
 * Copyright (c) 2022, jeejio Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author         Notes
 * 2023-2-7       zhengqian      the first version
*/

#ifndef DRV_RTC_H__
#define DRV_RTC_H__

#include "freertos/FreeRTOS.h"
#include "freertos/device.h"

#ifdef JEE_USING_ALARM
#include <alarm.h>
#endif


int jee_hw_rtc_init(void);

#endif

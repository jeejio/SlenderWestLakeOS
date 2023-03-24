/*
 * Copyright (c) 2022, jeejio Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 
 */

#ifndef __HAL_WLAN_LWIP_H__
#define __HAL_WLAN_LWIP_H__

#include "freertos/device.h"
#include "freertos/FreeRTOS.h"
#include "timers.h"
#include "hal_wlan_prot.h"
#include "hal_workqueue.h"
#include "ethernetif_port.h"


#ifdef __cplusplus
extern "C" {
#endif

struct lwip_prot_des
{
    struct rt_wlan_prot prot;
    struct eth_device eth;
    jee_int8_t connected_flag;
    // struct rt_timer timer;
    TimerHandle_t timer;
    struct rt_work work;
};


int rt_wlan_lwip_init(void);

#ifdef __cplusplus
}
#endif

#endif

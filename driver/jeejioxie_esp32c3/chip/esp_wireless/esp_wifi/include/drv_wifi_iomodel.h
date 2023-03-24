/*
 * SPDX-FileCopyrightText: 2015-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __DRV_WIFI_H__
#define __DRV_WIFI_H__


#include <hal_wlan_dev.h>
#include <hal_wlan_mgnt.h>


struct wlan_trans_ops {
    jee_err_t (*wlan_drv_input)(void *wlanif, void *buffer, size_t len, void* l2_buff);
    jee_err_t (*wlan_drv_output)(void *wlanif, void *pbuf);
};

// struct rt_wlan_info wlan_info[5]={0};

// struct rt_wlan_scan_result scan_result;


int wm_hw_wifi_init(void);
extern void example_disp_buf(char* title, char* buf, int length);

#endif /* __DRV_WIFI_H__ */

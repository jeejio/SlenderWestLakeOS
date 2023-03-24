/*
 * Copyright (c) 2022, jeejio Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 
 */

#ifndef __WLAN_CFG_H__
#define __WLAN_CFG_H__

#include "freertos/device.h"
#include "freertos/FreeRTOS.h"
#include <hal_wlan_dev.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef JEE_WLAN_CFG_INFO_MAX
#define JEE_WLAN_CFG_INFO_MAX    (3) /* min is 1 */
#endif

#define JEE_WLAN_CFG_MAGIC       (0x426f6d62)

struct jee_wlan_cfg_info
{
    struct rt_wlan_info info;
    struct rt_wlan_key key;
};

typedef int (*jee_wlan_wr)(void *buff, int len);

struct jee_wlan_cfg_ops
{
    int (*read_cfg)(void *buff, int len);
    int (*get_len)(void);
    int (*write_cfg)(void *buff, int len);
};

void jee_wlan_cfg_init(void);

void jee_wlan_cfg_set_ops(const struct jee_wlan_cfg_ops *ops);

int jee_wlan_cfg_get_num(void);

int jee_wlan_cfg_read(struct jee_wlan_cfg_info *cfg_info, int num);

int jee_wlan_cfg_read_index(struct jee_wlan_cfg_info *cfg_info, int index);

jee_err_t jee_wlan_cfg_save(struct jee_wlan_cfg_info *cfg_info);

jee_err_t jee_wlan_cfg_cache_refresh(void);

jee_err_t jee_wlan_cfg_cache_save(void);

int jee_wlan_cfg_delete_index(int index);

void jee_wlan_cfg_delete_all(void);

void jee_wlan_cfg_dump(void);

#ifdef __cplusplus
}
#endif

#endif

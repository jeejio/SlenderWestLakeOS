/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-10-14     wangqiang    the first version
 */

#ifndef __MDIO_H__
#define __MDIO_H__

#ifdef __cplusplus
extern "C"
{
#endif

struct jee_mdio_bus_ops
{
    jee_bool_t (*init)(void *bus, jee_uint32_t src_clock_hz);
    jee_size_t (*read)(void *bus, jee_uint32_t addr, jee_uint32_t reg, void *data, jee_uint32_t size);
    jee_size_t (*write)(void *bus, jee_uint32_t addr, jee_uint32_t reg, void *data, jee_uint32_t size);
    jee_bool_t (*uninit)(void *bus);
};

struct jee_mdio_bus
{
    void *hw_obj;
    char *name;
    struct jee_mdio_bus_ops *ops;
};

typedef struct jee_mdio_bus jee_mdio_t;

#ifdef __cplusplus
}
#endif

#endif

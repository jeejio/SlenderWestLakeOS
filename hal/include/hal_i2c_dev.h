/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author        Notes
 * 2012-04-25     weety         first version
 * 2021-04-20     RiceChen      added bus clock command
 */

#ifndef __I2C_DEV_H__
#define __I2C_DEV_H__

#include <freertos/jeedef.h>
#include <hal_i2c.h>


#ifdef __cplusplus
extern "C" {
#endif

#define RT_I2C_DEV_CTRL_10BIT        (JEE_DEVICE_CTRL_BASE(I2CBUS) + 0x01)
#define RT_I2C_DEV_CTRL_ADDR         (JEE_DEVICE_CTRL_BASE(I2CBUS) + 0x02)
#define RT_I2C_DEV_CTRL_TIMEOUT      (JEE_DEVICE_CTRL_BASE(I2CBUS) + 0x03)
#define RT_I2C_DEV_CTRL_RW           (JEE_DEVICE_CTRL_BASE(I2CBUS) + 0x04)
#define RT_I2C_DEV_CTRL_CLK          (JEE_DEVICE_CTRL_BASE(I2CBUS) + 0x05)

struct rt_i2c_priv_data
{
    struct jee_i2c_msg  *msgs;
    jee_size_t  number;
};

jee_err_t jee_i2c_bus_device_device_init(struct jee_i2c_bus_device *bus, const char *name);

#ifdef __cplusplus
}
#endif

#endif

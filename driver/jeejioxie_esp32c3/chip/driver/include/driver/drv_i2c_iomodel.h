/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-11-15     xckhmf       First Verison
 *
 */
#ifndef __DRV_I2C_H__
#define __DRV_I2C_H__

#ifdef __cplusplus
extern "C" {
#endif

#define SDA_GPIO (8)
#define SCL_GPIO (9)
#define I2C_DEV_NAME    "i2c0hard"
#define TAG "DHT20"

/*iic Éè±¸³õÊ¼»¯*/
#define JEE_IIC_BUS_CMD_INIT        (1)

extern int jee_hw_i2c_init(void);
extern void i2c_master_init(void);

#ifdef __cplusplus
}
#endif


#endif  /* __DRV_I2C_H__ */

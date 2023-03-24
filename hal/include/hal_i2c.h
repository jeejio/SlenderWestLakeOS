/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author        Notes
 * 2012-04-25     weety         first version
 * 2021-04-20     RiceChen      added support for bus control api
 */

#ifndef __I2C_H__
#define __I2C_H__

#include <freertos/device.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>



#ifdef __cplusplus
extern "C" {
#endif


#define JEE_I2C_RD               (1u << 0)
#define JEE_I2C_STOP             (1u << 1)
#define JEE_I2C_ADDR_10BIT       (1u << 2)  /* this is a ten bit chip address */
#define JEE_I2C_WR               (1u << 3)
#define JEE_I2C_START            (1u << 4)
#define JEE_I2C_IGNORE_NACK      (1u << 5)
#define JEE_I2C_NO_READ_ACK      (1u << 6)  /* when I2C reading, we do not ACK */
#define JEE_I2C_NO_STOP          (1u << 7)
#define JEE_I2C_NO_START         (1u << 8)



struct jee_i2c_msg
{
    jee_uint16_t addr;
    jee_uint16_t flags;
    jee_uint16_t len;
    jee_uint8_t  *buf;
};

struct jee_i2c_bus_device;

struct jee_i2c_bus_device_ops
{
    jee_size_t (*master_xfer)(struct jee_i2c_bus_device *bus,
                             struct jee_i2c_msg msgs[],
                             jee_uint32_t num);
    jee_size_t (*slave_xfer)(struct jee_i2c_bus_device *bus,
                            struct jee_i2c_msg msgs[],
                            jee_uint32_t num);
    jee_err_t (*i2c_bus_control)(struct jee_i2c_bus_device *bus,
                                int cmd ,
                                void *arg);
};

/*for i2c bus driver*/
struct jee_i2c_bus_device
{
    struct jee_device parent;
    const struct jee_i2c_bus_device_ops *ops;
    jee_uint16_t  flags;
    SemaphoreHandle_t lock;
    jee_uint32_t  timeout;
    jee_uint32_t  retries;
    void *priv;
};

struct rt_i2c_client
{
    struct jee_i2c_bus_device       *bus;
    jee_uint16_t                    client_addr;
};

jee_err_t jee_i2c_bus_device_register(struct jee_i2c_bus_device *bus,
                                    const char               *bus_name);
struct jee_i2c_bus_device *jee_i2c_bus_device_find(const char *bus_name);
jee_size_t jee_i2c_transfer(struct jee_i2c_bus_device *bus,
                          struct jee_i2c_msg         msgs[],
                          jee_uint32_t               num);
jee_err_t jee_i2c_control(struct jee_i2c_bus_device *bus,
                        jee_uint32_t               cmd,
                        jee_uint32_t               arg);
jee_size_t jee_i2c_master_send(struct jee_i2c_bus_device *bus,
                             jee_uint16_t               addr,
                             jee_uint16_t               flags,
                             const jee_uint8_t         *buf,
                             jee_uint32_t               count);
jee_size_t jee_i2c_master_recv(struct jee_i2c_bus_device *bus,
                             jee_uint16_t               addr,
                             jee_uint16_t               flags,
                             jee_uint8_t               *buf,
                             jee_uint32_t               count);


jee_inline jee_err_t jee_i2c_bus_lock(struct jee_i2c_bus_device *bus, jee_tick_t timeout)
{
    // return rt_mutex_take(&bus->lock, timeout);

    return xSemaphoreTake(bus->lock, timeout);
}
jee_inline jee_err_t jee_i2c_bus_unlock(struct jee_i2c_bus_device *bus)
{
    // return rt_mutex_release(&bus->lock);

    return xSemaphoreGive(bus->lock);
    
}

int jee_i2c_core_init(void);

#ifdef __cplusplus
}
#endif

#endif

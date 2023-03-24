/*
 * Copyright (c) 2022, jeejio Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-1-10      zhengqian    first version
 */


#ifndef __SPI_BIT_OPS_H__
#define __SPI_BIT_OPS_H__

#include <freertos/device.h>

#ifdef __cplusplus
extern "C" {
#endif

struct jee_spi_bit_ops
{
    void *data;            /* private data for lowlevel routines */
    void (*const tog_sclk)(void *data);
    void (*const set_sclk)(void *data, jee_int32_t state);
    void (*const set_mosi)(void *data, jee_int32_t state);
    void (*const set_miso)(void *data, jee_int32_t state);
    jee_int32_t (*const get_sclk)(void *data);
    jee_int32_t (*const get_mosi)(void *data);
    jee_int32_t (*const get_miso)(void *data);

    void (*const dir_mosi)(void *data, jee_int32_t state);
    void (*const dir_miso)(void *data, jee_int32_t state);

    void (*const udelay)(jee_uint32_t us);
    jee_uint32_t delay_us;  /* sclk, mosi and miso line delay */
};

struct jee_spi_bit_obj
{
    struct jee_spi_bus bus;
    struct jee_spi_bit_ops *ops;
    struct jee_spi_configuration config;
};

jee_err_t jee_spi_bit_add_bus(struct jee_spi_bit_obj *obj,
                            const char            *bus_name,
                            struct jee_spi_bit_ops *ops);

#ifdef __cplusplus
}
#endif

#endif

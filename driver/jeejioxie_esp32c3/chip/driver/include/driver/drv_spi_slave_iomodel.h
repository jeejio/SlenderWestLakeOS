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
#ifndef __DRV_SPI_SLAVE_H__
#define __DRV_SPI_SLAVE_H__

#ifdef __cplusplus
extern "C" {
#endif
#include <jeedef.h>
#include <spi_common.h>
#include <driver/gpio.h>

#define GPIO_HANDSHAKE 3
#define PIN_NUM_MISO 2
#define PIN_NUM_MOSI 7
#define PIN_NUM_CLK 6
#define PIN_NUM_CS 10
#define SPI_BUS_NAME    "spi2"
#define SPI_DEV_NAME    "spi20"
#define SLAVE_HOST SPI2_HOST


extern int jee_spi_slave_init(void);
extern jee_err_t esp_spi_bus_attach_device(const char *bus_name, const char *device_name, jee_uint32_t pin);

#ifdef __cplusplus
}
#endif


#endif  /* __DRV_I2C_H__ */
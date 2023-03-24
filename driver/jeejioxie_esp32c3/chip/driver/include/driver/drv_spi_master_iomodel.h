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
#ifndef __DRV_SPI_MASTER_H__
#define __DRV_SPI_MASTER_H__

#ifdef __cplusplus
extern "C" {
#endif
#include <jeedef.h>
#include <spi_common.h>
#include <driver/spi_master.h>
#include <driver/gpio.h>

#define SPI_BUS_NAME    "spi2"

#define GPIO_HANDSHAKE 3
#define SPI_BUS_TEST_DEFAULT_CONFIG() {\
        .miso_io_num=2, \
        .mosi_io_num=7,\
        .sclk_io_num=6,\
        .quadwp_io_num=-1,\
        .quadhd_io_num=-1,\
        .max_transfer_sz=240*320 \
    }
#define SPI_DEVICE_TEST_DEFAULT_CONFIG() {\
        .command_bits=0, \
        .address_bits=0, \
        .dummy_bits=0, \
        .clock_speed_hz=20000000, \
        .duty_cycle_pos=128,      \
        .mode=0, \
        .spics_io_num= -1, \
        .queue_size=7, \
        .flags=0, \
        .pre_cb=NULL, \
        .post_cb=NULL\
    }

typedef struct{
    spi_bus_config_t *spi_bus;
    spi_device_interface_config_t *spi_device;
} spi_bus_device_configuration_t;


extern int jee_spi_bus_init(void);
extern jee_err_t esp_spi_bus_attach_device(const char *bus_name, const char *device_name, jee_uint32_t pin);

#ifdef __cplusplus
}
#endif


#endif  /* __DRV_I2C_H__ */
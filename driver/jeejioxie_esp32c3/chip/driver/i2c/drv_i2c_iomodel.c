/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-11-15     xckhmf       First Verison
 * 2021-11-27     chenyingchun fix _master_xfer bug
 *
 */

#include <hal_i2c.h>
#include "driver/i2c.h"
#include <driver/gpio.h>
#include <hal/i2c_types.h>
#include "freertos/semphr.h"
#include <jeedef.h>
#include <driver/drv_i2c_iomodel.h>
#include "esp_log.h"
#include "auto_init.h"
// #include <esp_log.h>
// #include <esp_err.h>



static struct jee_i2c_bus_device i2c0_bus;


int jee_hw_i2c_init(void);
void i2c_master_init(void);


void i2c_master_init(void)
{

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_GPIO, // select GPIO specific to your project
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = SCL_GPIO, // select GPIO specific to your project
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000, // select frequency specific to your project
        .clk_flags = 0,             /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
    };

    i2c_param_config(I2C_NUM_0, &conf);
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
}
static i2c_cmd_handle_t iic_cmd;
static jee_size_t _master_xfer(struct jee_i2c_bus_device *bus,
                              struct jee_i2c_msg         *msgs,
                              jee_uint32_t               num)
{
    struct jee_i2c_msg *msg;
    jee_int32_t i;
    esp_err_t espRc;

    for (i = 0; i < num; i++)
    {
        msg = &msgs[i];

        if (msg->flags & JEE_I2C_RD)
        {
            if ((msg->flags & JEE_I2C_NO_START) == 0)
            {
                iic_cmd = i2c_cmd_link_create();
                assert(iic_cmd != NULL);
                i2c_master_start(iic_cmd);
            }
            if ((msg->flags & JEE_I2C_START))
            {
                i2c_master_start(iic_cmd);
            }

            i2c_master_write_byte(iic_cmd, msg->addr << 1 | I2C_MASTER_READ, true);
            i2c_master_read(iic_cmd, msg->buf, msg->len, I2C_MASTER_LAST_NACK);

            if (msg->flags & JEE_I2C_STOP)
            {
                i2c_master_stop(iic_cmd);
            }
            if ((msg->flags & JEE_I2C_NO_STOP) == 0)
            {
                espRc = i2c_master_cmd_begin(I2C_NUM_0, iic_cmd, 10 / portTICK_PERIOD_MS);

                i2c_cmd_link_delete(iic_cmd);

                if (espRc == ESP_OK)
                {
                    LogDebug(TAG, "read data ok");
                    return 1;
                }
                else
                {
                    LogDebug(TAG, "write data fail");
                    goto out;
                }
            }
        }
        else if (msg->flags & JEE_I2C_WR)
        {
            if ((msg->flags & JEE_I2C_NO_START) == 0)
            {
                iic_cmd = i2c_cmd_link_create();
                assert(iic_cmd != NULL);
                i2c_master_start(iic_cmd);
            }
            if ((msg->flags & JEE_I2C_START))
            {
                i2c_master_start(iic_cmd);
            }

            i2c_master_write_byte(iic_cmd, msg->addr << 1 | I2C_MASTER_WRITE, true);

            i2c_master_write(iic_cmd, msg->buf, msg->len, true);
            if (msg->flags & JEE_I2C_STOP)
            {
                i2c_master_stop(iic_cmd);
            }
            if ((msg->flags & JEE_I2C_NO_STOP) == 0)
            {
                espRc = i2c_master_cmd_begin(I2C_NUM_0, iic_cmd, 10 / portTICK_PERIOD_MS);
                i2c_cmd_link_delete(iic_cmd);
                if (espRc == ESP_OK)
                {
                    LogDebug(TAG, "write data ok buf:0x%x ", *(msg->buf));
                    return 1;
                }
                else
                {
                    LogDebug(TAG, "write data fail");
                    goto out;
                }
            }
        }
    }

out:
    iic_cmd = i2c_cmd_link_create();
    i2c_master_stop(iic_cmd);
    i2c_master_cmd_begin(I2C_NUM_0, iic_cmd, 10 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(iic_cmd);

    return 0;
}

jee_err_t i2c_bus_control(struct jee_i2c_bus_device *bus,
                          int cmd,
                          void *arg)
{
    switch (cmd)
    {
    case JEE_IIC_BUS_CMD_INIT:
    {
        i2c_master_init();
    }
        break;
    default:
        break;
    }
    return 0;
}

static const struct jee_i2c_bus_device_ops _i2c_ops =
    {
        _master_xfer,
        NULL,
        i2c_bus_control,
};


int jee_hw_i2c_init(void)
{
    i2c0_bus.ops= &_i2c_ops;
    i2c0_bus.timeout = 0;

    return jee_i2c_bus_device_register(&i2c0_bus, I2C_DEV_NAME);
}
INIT_PREV_EXPORT(jee_hw_i2c_init);

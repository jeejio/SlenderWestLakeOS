/*
 * Copyright (c) 2022, jeejio Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author         Notes
 * 2022-12-13     zhengqian      the first version
*/

#ifndef __JEE_SENSOR_DHT20_H__
#define __JEE_SENSOR_DHT20_H__

#include "device.h"
#include "hal_sensor.h"    //sesor IO模型驱动框架头文件
#include "dht20.h"         //厂家驱动头文件
#include "string.h"
#include "auto_init.h"
#include "driver/i2c.h"



jee_size_t sensor_fetch_data(struct jee_sensor_device *sensor, void *buf, jee_size_t len)
{
    
     jee_int32_t *pbuf = buf;
    if(buf == NULL)
        return 0;
   
    dht20_get_data(pbuf); 
     
    return (0);
}

jee_err_t sensor_control(struct jee_sensor_device *sensor, int cmd, void *arg)
{
    printf("DTH20_sensor_control enter [cmd ] %d\n", cmd);

    switch (cmd)
    {

    case JEE_SENSOR_CTRL_SET_POWER:
    {
        int power_mode_type = (int)arg;
        switch (power_mode_type)
        {
        case JEE_SENSOR_POWER_NORMAL:
        {
            printf(" start : DTH20 \n");
            dht20_init();
            printf(" end   : DTH20 \n");
        }
        break;
        case JEE_SENSOR_POWER_DOWN:
        {
        }
        break;
        default:
            break;
        }
    }

    break;

    default:
        break;
    }

    return JEE_EOK;
}


static struct jee_sensor_ops sensor_ops =
{
    sensor_fetch_data,
    sensor_control
};


int sensor_DHT20_init(const char *name, struct jee_sensor_config *cfg)
{
    jee_int8_t result;
    jee_sensor_t sensor_temp_humi = JEE_NULL;
    
     /* sensor register */
    sensor_temp_humi = pvPortMalloc(sizeof(struct jee_sensor_device));
    if (sensor_temp_humi == JEE_NULL)
        return -1;

    memset(sensor_temp_humi, 0, sizeof(struct jee_sensor_device));

    sensor_temp_humi->info.type       = JEE_SENSOR_CLASS_TEMP;
    sensor_temp_humi->info.vendor     = JEE_SENSOR_VENDOR_UNKNOWN;
    sensor_temp_humi->info.model      = "DHT20";
    //sensor_temp_humi->info.unit       = RT_SENSOR_UNIT_DCELSIUS;
    sensor_temp_humi->info.intf_type  = JEE_SENSOR_INTF_I2C;
    //sensor_temp_humi->info.range_max  = SENSOR_TEMP_RANGE_MAX;
    //sensor_temp_humi->info.range_min  = SENSOR_TEMP_RANGE_MIN;

    if(cfg != JEE_NULL)
    	memcpy(&sensor_temp_humi->config, cfg, sizeof(struct jee_sensor_config));
    	
    sensor_temp_humi->ops = &sensor_ops;

    result = jee_hw_sensor_register(sensor_temp_humi, name, JEE_DEVICE_FLAG_RDONLY, JEE_NULL);
    if (result != JEE_EOK)
    {
        //LOG_E("device register err code: %d", result);
        goto __exit;
    }

 
    return JEE_EOK;
    
__exit:
    if (sensor_temp_humi)
        vPortFree(sensor_temp_humi);
   
    return -JEE_ERROR;     
}



// #define SDA_GPIO    (8)
// #define SCL_GPIO    (9)
// void i2c_master_init(void)
// {
//     i2c_config_t conf = {
// 		.mode = I2C_MODE_MASTER,
// 		.sda_io_num = SDA_GPIO,         // select GPIO specific to your project
// 		.sda_pullup_en = GPIO_PULLUP_DISABLE,
// 		.scl_io_num = SCL_GPIO,         // select GPIO specific to your project
// 		.scl_pullup_en = GPIO_PULLUP_DISABLE,
// 		.master.clk_speed = 100000,  // select frequency specific to your project
// 		.clk_flags = 0,          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
// 	};

//     i2c_param_config(I2C_NUM_0, &conf);
// 	i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
// }

#define DHT20_ADDR          (0x38)
#define I2C_DEV_Name        "i2c0hard"
int jee_hw_sensor_DHT20_init()
{
    //printf("begin to execute jee_hw_sensor_DHT20_init\n");
    //i2c_master_init();

    struct jee_sensor_config cfg = {0};
    cfg.intf.dev_name  = I2C_DEV_Name;
    cfg.intf.user_data = (void *)DHT20_ADDR;
    sensor_DHT20_init("DHT20", &cfg);

    //printf("finish to execute jee_hw_sensor_DHT20_init\n");
    return JEE_EOK;
}

INIT_DEVICE_EXPORT(jee_hw_sensor_DHT20_init);

#endif

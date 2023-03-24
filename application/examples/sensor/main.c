/*
 * Copyright (c) 2022, jeejio Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 20222.12.15    zhengqian    first version
 */
 
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/device.h"
#include "hal_sensor.h"
//#include "driver/i2c.h"
#include "auto_init.h"

//本无需包含该头文件，这里暂时模拟系统启动初始化，自动加载jee_hw_sensor_DHT20_init函数
//#include "drv_sensor_DHT20_IoModel.h"

//#define DHT20_ADDR          (0x38)
//#define I2C_DEV_Name        "i2c0hard"

//static const char *TAG = "sensor_Model example"; // 用于log

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


jee_device_t sensor_dev = NULL;


// main
void app_main(void)
{
    printf("sensor example Start....\n");
    rt_components_init();

    //i2c_master_init();
    //vTaskDelay(1000 / portTICK_PERIOD_MS);

    // 模拟启动注册，替代：INIT_BOARD_EXPORT(jee_hw_sensor_DHT20_init);
    //jee_hw_sensor_DHT20_init();
    //vTaskDelay(1*1000 / portTICK_PERIOD_MS);
    //printf("register the sensor Model done\n");

    // 查找设备
    sensor_dev = jee_device_find("temp_DHT20");
    if (sensor_dev == JEE_NULL)
    {
        printf("can not find sensor Model\n");
        return;
    }
    else
        printf("find sensor Model ok\n");


    // 打开设备
    jee_err_t result = JEE_EOK;
    result = jee_device_open(sensor_dev, JEE_DEVICE_FLAG_RDONLY);
    if (result != JEE_EOK)
    {
        printf("can not open senor device\n");
        return;
    }
    else
        printf("open senor device ok\n");

    // 初始化设备
    jee_device_control(sensor_dev, JEE_SENSOR_CTRL_USER_CMD_START + 1, NULL);
    vTaskDelay(200 / portTICK_PERIOD_MS);

    // 读写设备：获取温度、湿度数据
    int ntemp,nhumi;
    uint32_t databuf[2] = {0};
    while (1) 
    {
        jee_device_read(sensor_dev, NULL, (void *)databuf, sizeof(databuf));

        nhumi = (databuf[0] * 100 * 10 / 1024 / 1024) / 10;
        ntemp = (databuf[1] * 200 * 10 / 1024 / 1024 - 500) / 10;

        printf("温度：%d    湿度： %d\n", ntemp, nhumi);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}



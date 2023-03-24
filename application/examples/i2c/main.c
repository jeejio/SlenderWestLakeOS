/*
 * Copyright (c) 2022, jeejio Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 *
 */
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "jeedef.h"
#include <device.h>
#include "hal_i2c.h"
#include "driver/i2c.h"
#include "auto_init.h"


#define DHT20_ADDR      (0x38)
#define NONE            0
#define I2C_DEV_NAME    "i2c0hard"
#define TAG "DHT20"


typedef union pos{
    struct msg
    {
        jee_off_t addr:16;
        jee_off_t flag:16;
    }msg;
    jee_off_t pos;
}pos_t;


pos_t tt={};
jee_device_t i2c_dev;
extern SemaphoreHandle_t xMutex;


void jee_config_pos(pos_t *tt, jee_uint16_t flag, jee_uint16_t addr);
void jee_set_value(jee_int8_t *value, jee_int8_t dat);
void jee_dht20_send_ac(void);
void jee_dht20_init();



void app_main(void)
{
    uint8_t data[6] = {0};
    uint32_t humi_temp_data[2] = {0};
    int temperature;
    int humidity;


    rt_components_init();

    
    //设备查找和打开
    i2c_dev = jee_device_find(I2C_DEV_NAME);
    if (i2c_dev == NONE)
    {
        LogError(TAG, "can not find i2c_dev: %s", I2C_DEV_NAME);
        return ;
    }else 
    {
        LogInfo(TAG, "find i2c_dev: %s", i2c_dev->name);
    }


    if (jee_device_open(i2c_dev, JEE_DEVICE_FLAG_RDONLY) != JEE_EOK)
    {
        LogError(TAG, "open i2c_dev failed!");
        return ;
    }else 
    {
        LogInfo(TAG, "open i2c_dev ok");
    }

    //dht20初始化
    jee_dht20_init();


    while(1)
    {
        jee_dht20_send_ac();
        vTaskDelay(pdMS_TO_TICKS(80));


        //读取传感器参数
        jee_config_pos(&tt, JEE_I2C_RD | JEE_I2C_START, DHT20_ADDR);
        jee_device_read(i2c_dev, tt.pos, &data[0], 1);       


        jee_config_pos(&tt, JEE_I2C_RD, NONE);
        jee_device_read(i2c_dev, tt.pos, &data[1], 1);


        jee_config_pos(&tt, JEE_I2C_RD, NONE);
        jee_device_read(i2c_dev, tt.pos, &data[2], 1);

        jee_config_pos(&tt, JEE_I2C_RD, NONE);
        jee_device_read(i2c_dev, tt.pos, &data[3], 1);

        jee_config_pos(&tt, JEE_I2C_RD, NONE);
        jee_device_read(i2c_dev, tt.pos, &data[4], 1);

        jee_config_pos(&tt, JEE_I2C_RD, NONE);
        jee_device_read(i2c_dev, tt.pos, &data[5], 1);

 
        humi_temp_data[0] = ((data[1] << 16) + (data[2] << 8) + (data[3])) >> 4;      // humidity
        humi_temp_data[1] = ((data[3] << 16) + (data[4] << 8) + (data[5])) & 0xfffff; // temperature

        humidity = (humi_temp_data[0] * 100 * 10 / 1024 / 1024) / 10;
        temperature = (humi_temp_data[1] * 200 * 10 / 1024 / 1024 - 500); 


        printf("hu:%d   temp:%d \r\n", humidity, temperature);
        vTaskDelay(pdMS_TO_TICKS(1000));

    }
}


void jee_dht20_init()
{
    jee_int8_t value=0;

    jee_config_pos(&tt, JEE_I2C_WR | JEE_I2C_START, DHT20_ADDR);
    jee_set_value(&value, 0xa8);
    jee_device_write(i2c_dev, tt.pos, &value, 1);

    jee_config_pos(&tt, JEE_I2C_WR, NONE);
    jee_set_value(&value, 0x00);
    jee_device_write(i2c_dev, tt.pos, &value, 1);

    jee_config_pos(&tt, JEE_I2C_STOP | JEE_I2C_WR, NONE);
    jee_set_value(&value, 0x00);
    jee_device_write(i2c_dev, tt.pos, &value, 1);

    vTaskDelay(pdMS_TO_TICKS(10));

    jee_config_pos(&tt, JEE_I2C_WR | JEE_I2C_START, DHT20_ADDR);
    jee_set_value(&value, 0xbe);
    jee_device_write(i2c_dev, tt.pos, &value, 1);

    jee_config_pos(&tt, JEE_I2C_WR, NONE);
    jee_set_value(&value, 0x08);
    jee_device_write(i2c_dev, tt.pos, &value, 1);

    jee_config_pos(&tt, JEE_I2C_STOP | JEE_I2C_WR, NONE);
    jee_set_value(&value, 0x00);
    jee_device_write(i2c_dev, tt.pos, &value, 1);

    LogDebug(TAG, "jee_dht20_init ok");

}

void jee_dht20_send_ac(void)  
{
    jee_int8_t value=0;

    jee_config_pos(&tt, JEE_I2C_WR | JEE_I2C_START, DHT20_ADDR);
    jee_set_value(&value, 0xac);
    jee_device_write(i2c_dev, tt.pos, &value, 1);

    jee_config_pos(&tt, JEE_I2C_WR, NONE);
    jee_set_value(&value, 0x33);
    jee_device_write(i2c_dev, tt.pos, &value, 1);

    jee_config_pos(&tt, JEE_I2C_STOP | JEE_I2C_WR, NONE);
    jee_set_value(&value, 0x00);
    jee_device_write(i2c_dev, tt.pos, &value, 1);

    LogDebug(TAG, "jee_dht20_send_ac ok");
}


void jee_config_pos(pos_t *tt, jee_uint16_t flag, jee_uint16_t addr)
{
    bzero(&(tt->pos), sizeof(tt->pos));
    tt->msg.flag = tt->msg.flag | flag;
    tt->msg.addr = tt->msg.addr | addr;
}


void jee_set_value(jee_int8_t *value, jee_int8_t dat)
{
    *value=dat;
}



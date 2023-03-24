/*
 * Copyright (c) 2022, jeejio Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author         Notes
 * 2023-02-01     xuyuhu         the first version
 */

#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tal_steering_gear.h"
#include "freertos/device.h"
#include "hal_sensor.h"

jee_device_t sg_sensor_dev = NULL;
SteeringGearConfig_t SensorSG90Config;

int lTalSteeringGearGetRotationAngle(void)
{
    uint32_t databuf[2] = {0};
    jee_device_read(sg_sensor_dev, NULL, (void *)databuf, sizeof(databuf));
    SensorSG90Config.current_dirction = databuf[0];
    SensorSG90Config.current_angle = databuf[1];
    return SensorSG90Config.current_angle;
}

void vTalSteeringGearSetRotationAngle(int angel)
{
    SensorSG90Config.set_angle = angel;

    {
        jee_device_control(sg_sensor_dev, JEE_SENSOR_CTRL_USER_CMD_START + 1, &SensorSG90Config.set_angle);
    }
}

int lTalSteeringGearGetRotationDirction(void)
{
    uint32_t databuf[2] = {0};
    jee_device_read(sg_sensor_dev, NULL, (void *)databuf, sizeof(databuf));
    SensorSG90Config.current_dirction = databuf[0];
    SensorSG90Config.current_angle = databuf[1];
    return SensorSG90Config.current_dirction;
}

void vTalSteeringGearSetRotationDirction(int valule)
{
    if (SensorSG90Config.current_dirction != valule)
    {
        SensorSG90Config.set_dirction = valule;
        /*方向切换后：正向30° == 反向 150°*/
        SensorSG90Config.set_angle = 180 - SensorSG90Config.set_angle;
        SensorSG90Config.current_angle = SensorSG90Config.set_angle;
        {
            jee_device_control(sg_sensor_dev, JEE_SENSOR_CTRL_USER_CMD_START + 2, &SensorSG90Config.set_dirction);
        }
    }
}

int vTalSteeringGearInit(void)
{
    // 查找设备
    sg_sensor_dev = jee_device_find("sg_SG90");
    if (sg_sensor_dev == JEE_NULL)
    {
        printf("can not find sensor Model\n");
        return -1;
    }
    else
        printf("find sensor Model ok\n");

    // 打开设备
    jee_err_t result = JEE_EOK;
    result = jee_device_open(sg_sensor_dev, JEE_DEVICE_FLAG_RDONLY);
    if (result != JEE_EOK)
    {
        printf("can not open senor device\n");
        return -1;
    }
    else
        printf("open senor device ok\n");
    return 0;
}

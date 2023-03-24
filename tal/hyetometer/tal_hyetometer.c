/*
 * Copyright (c) 2022, jeejio Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author         Notes
 * 2023-02-10     xuyuhu         the first version
 */

#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tal_hyetometer.h"
#include "freertos/device.h"
#include "hal_sensor.h"

jee_device_t hyet_sensor_dev = NULL;
static HyetomterInfo_t GetInfo;

HyetomterInfo_t * xTalHyetometerOnReceive(void)
{
    jee_device_read(hyet_sensor_dev,NULL,&GetInfo,sizeof(GetInfo));
    return &GetInfo;
}

int cTalHyetometerGetIsRaining(void)
{
    xTalHyetometerOnReceive();
    return GetInfo.isRaining;
}

int cTalHyetometerGetPecipitationLevel(void)
{
    xTalHyetometerOnReceive();
    return GetInfo.precipitationLevel;
}

int vTalHyetometerInit(void)
{
    // 查找设备
    hyet_sensor_dev = jee_device_find("hyetometer_MK002");
    if (hyet_sensor_dev == JEE_NULL)
    {
        printf("can not find sensor Model\n");
        return -1;
    }
    else
        printf("find sensor Model ok\n");

    // 打开设备
    jee_err_t result = JEE_EOK;
    result = jee_device_open(hyet_sensor_dev, JEE_DEVICE_FLAG_RDONLY);
    if (result != JEE_EOK)
    {
        printf("can not open senor device\n");
        return -1;
    }
    else
        printf("open senor device ok\n");
    return 0;
}

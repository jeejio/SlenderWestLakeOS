/*
 * Copyright (c) 2022, jeejio Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author         Notes
 * 2023-02-03     xuyuhu         the first version
 */

#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tal_rotary_encoder.h"
#include "freertos/device.h"
#include "hal_sensor.h"

jee_device_t re_sensor_dev = NULL;

RotaryEncoderStatus_t GetStatus;

RotaryEncoderStatus_t * xTalRotaryEncoderOnReceive(void)
{
    uint32_t databuf[3] = {0};
    jee_device_read(re_sensor_dev, NULL, (void *)databuf, sizeof(databuf));
    GetStatus.RelativePos = databuf[0];
    GetStatus.IsPressed   = databuf[1];
    GetStatus.RotationDirection =databuf[2];
    return &GetStatus;
}

int lTalRotaryEncoderGetPosition(void)
{
    xTalRotaryEncoderOnReceive();
    return GetStatus.RelativePos;
}

bool cTalRotaryEncoderGetIsPressed(void)
{
    xTalRotaryEncoderOnReceive();
    return GetStatus.IsPressed;
}

bool cTalRotaryEncoderGetRotationDirection(void)
{
    xTalRotaryEncoderOnReceive();
    return GetStatus.RotationDirection;
}


int vTalRotaryEncoderInit(void)
{
    // 查找设备
    re_sensor_dev = jee_device_find("re_EC11");
    if (re_sensor_dev == JEE_NULL)
    {
        printf("can not find sensor Model\n");
        return -1;
    }
    else
        printf("find sensor Model ok\n");

    // 打开设备
    jee_err_t result = JEE_EOK;
    result = jee_device_open(re_sensor_dev, JEE_DEVICE_FLAG_RDONLY);
    if (result != JEE_EOK)
    {
        printf("can not open senor device\n");
        return -1;
    }
    else
        printf("open senor device ok\n");
    return 0;
}

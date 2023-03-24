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
#include "tal_joystick.h"
#include "freertos/device.h"
#include "hal_sensor.h"

jee_device_t joyst_sensor_dev = NULL;
static TalJoystickInfo_t GetInfo;
static TalJoystickPositionInfo_t GetPos;
void lTalJoyStickGetInfo(void)
{
    jee_device_read(joyst_sensor_dev, NULL, &GetInfo, sizeof(GetInfo));
}
int lTalJoystickGetStatus(void)
{
    lTalJoyStickGetInfo();
    return GetInfo.IsPress;
}

int lTalJoystickModeReceive(void)
{
    lTalJoyStickGetInfo();
    jee_device_control(joyst_sensor_dev,JEE_SENSOR_CTRL_USER_CMD_START + 1, NULL);
    return GetInfo.IsMode;
}

int lTalJoystickGetPositioninformationX(void)
{
    lTalJoyStickGetInfo();
    return GetInfo.getPositioninformationX;
}

int lTalJoystickGetPositioninformationY(void)
{
    lTalJoyStickGetInfo();
    return GetInfo.getPositioninformationY;
}

TalJoystickPositionInfo_t *lTalJoystickPositionReceive(void)
{
    lTalJoyStickGetInfo();
    GetPos.getPositioninformationX = GetInfo.getPositioninformationX;
    GetPos.getPositioninformationY = GetInfo.getPositioninformationY;
    return &GetPos;
}

int vTalJoystickInit(void)
{
    // 查找设备
    joyst_sensor_dev = jee_device_find("joystick_KY023");
    if (joyst_sensor_dev == JEE_NULL)
    {
        printf("can not find sensor Model\n");
        return -1;
    }
    else
        printf("find sensor Model ok\n");

    // 打开设备
    jee_err_t result = JEE_EOK;
    result = jee_device_open(joyst_sensor_dev, JEE_DEVICE_FLAG_RDONLY);
    if (result != JEE_EOK)
    {
        printf("can not open senor device\n");
        return -1;
    }
    else
        printf("open senor device ok\n");
    return 0;
}

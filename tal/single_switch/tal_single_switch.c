/*
 * Copyright (c) 2022, jeejio Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author         Notes
 * 2023-03-23     xuyuhu         the first version
 */

#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tal_single_switch.h"
#include "freertos/device.h"
#include "hal_sensor.h"
#include "auto_init.h"

jee_device_t single_switch_sensor_dev = NULL;
static TalSwitchInfo_t GetInfo;
void lTalSingleSwitchGetInfo(void)
{
    jee_device_read(single_switch_sensor_dev, NULL, &GetInfo, sizeof(GetInfo));
}
int lTalSingleSwitchGetStatus(void)
{
    lTalSingleSwitchGetInfo();
    return GetInfo.IsPress;
}

int lTalSingleSwitchModeReceive(void)
{
    lTalSingleSwitchGetInfo();
    jee_device_control(single_switch_sensor_dev,JEE_SENSOR_CTRL_USER_CMD_START + 1, NULL);
    return GetInfo.IsMode;
}

int vTalSingleSwitchInit(void)
{
    // 查找设备
    single_switch_sensor_dev = jee_device_find("button665_SWITCH");
    if (single_switch_sensor_dev == JEE_NULL)
    {
        printf("can not find sensor Model\n");
        return -1;
    }
    else
        printf("find sensor Model ok\n");

    // 打开设备
    jee_err_t result = JEE_EOK;
    result = jee_device_open(single_switch_sensor_dev, JEE_DEVICE_FLAG_RDONLY);
    if (result != JEE_EOK)
    {
        printf("can not open senor device\n");
        return -1;
    }
    else
        printf("open senor device ok\n");
    return 0;
}

INIT_APP_EXPORT(vTalSingleSwitchInit);

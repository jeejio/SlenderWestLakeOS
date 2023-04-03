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
#include "tal_button_panel.h"
#include "freertos/device.h"
#include "hal_sensor.h"

jee_device_t button_sensor_dev = NULL;
static ButtonPanelInfo_t ButtonPanelInfo ={
    .code =1,
    .button_mode =0,
    .value=0,
};
ButtonPanelInfo_t * cTalButtonPanelOnReceive(void)
{
    int buf[2];
    jee_device_read(button_sensor_dev,NULL,(void *)buf,sizeof(buf));
    ButtonPanelInfo.button_mode=buf[1];
    ButtonPanelInfo.value=buf[0];
    return &ButtonPanelInfo;
}


int  cTalButtonPanelGetButtonCodes(void)
{
    cTalButtonPanelOnReceive();
    return ButtonPanelInfo.value;
}

int cTalButtonPanelGetButtonMode(void)
{
    cTalButtonPanelOnReceive();
    return ButtonPanelInfo.button_mode;
}

int vTalButtonPanelInit(void)
{
    // 查找设备
    button_sensor_dev = jee_device_find("button_BT665");
    if (button_sensor_dev == JEE_NULL)
    {
        printf("can not find sensor Model\n");
        return -1;
    }
    else
        printf("find sensor Model ok\n");

    // 打开设备
    jee_err_t result = JEE_EOK;
    result = jee_device_open(button_sensor_dev, JEE_DEVICE_FLAG_RDONLY);
    if (result != JEE_EOK)
    {
        printf("can not open senor device\n");
        return -1;
    }
    else
        printf("open senor device ok\n");
    return 0;
}

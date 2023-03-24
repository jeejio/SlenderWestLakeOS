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
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tal_rfid.h"
#include "freertos/device.h"
#include "hal_sensor.h"

jee_device_t rfid_sensor_dev = NULL;

static uint8_t get_data[20];
uint8_t *vTalRfidGetCardInfo(void)
{
    memset(get_data, 0, sizeof(get_data));
    jee_device_read(rfid_sensor_dev, 0, get_data, sizeof(get_data));
    return get_data;
}

void vTalRfidGetID(uint8_t *id)
{
    vTalRfidGetCardInfo();
    memcpy(id, get_data, 4);
}

void vTalRfidOnReceive(uint8_t *id, uint8_t *dat)
{
    vTalRfidGetCardInfo();
    memcpy(id, get_data, 4);
    memcpy(dat, get_data + 4, 16);
}

void vTalRfidGetData(uint8_t *dat)
{
    vTalRfidGetCardInfo();
    memcpy(dat, get_data + 4, 16);
}

void vTalRfidSetData(uint8_t *data)
{
    jee_device_control(rfid_sensor_dev, JEE_SENSOR_CTRL_USER_CMD_START + 1, data);
}

void vTalRfidInit(void)
{
    // 查找设备
    rfid_sensor_dev = jee_device_find("rfid_RC522");
    if (rfid_sensor_dev == JEE_NULL)
    {
        printf("can not find sensor Model\n");
        return;
    }
    else
        printf("find sensor Model ok\n");

    // 打开设备
    jee_err_t result = JEE_EOK;
    result = jee_device_open(rfid_sensor_dev, JEE_DEVICE_FLAG_RDONLY);
    if (result != JEE_EOK)
    {
        printf("can not open senor device\n");
        return;
    }
    else
        printf("open senor device ok\n");
}

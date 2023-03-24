/*
 * Copyright (c) 2022, jeejio Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author         Notes
 * 2023-02-07     xuyuhu         the first version
 */

#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tal_six_axis.h"
#include "freertos/device.h"
#include "hal_sensor.h"

jee_device_t six_axis_sensor_dev = NULL;

SixAxisAccelerometer_t AccStatus;
SixAxisGyroscope_t  GyrStatus;
SixAxisEulerAngle_t EulerStatus;
static SixAxisOnReceive_t  OnReceive;

SixAxisGyroscope_t * vTalSixAxisGetGyroscope(void)
{
    float databuf[9] = {0};
    jee_device_read(six_axis_sensor_dev, NULL, (void *)databuf, sizeof(databuf));
    GyrStatus.angularVelocityX = databuf[3];
    GyrStatus.angularVelocityY = databuf[4];
    GyrStatus.angularVelocityZ = databuf[5];
    return &GyrStatus;
}

SixAxisEulerAngle_t * vTalSixAxisGetEulerAngle(void)
{
    float databuf[9] = {0};
    jee_device_read(six_axis_sensor_dev, NULL, (void *)databuf, sizeof(databuf));
    EulerStatus.pithAngle = databuf[6];
    EulerStatus.rollAngle = databuf[7];
    EulerStatus.yawAngle = databuf[8];
    return &EulerStatus;
}
SixAxisAccelerometer_t * vTalSixAxisGetAccelerometer(void)
{
    float databuf[9] = {0};
    jee_device_read(six_axis_sensor_dev, NULL, (void *)databuf, sizeof(databuf));
    AccStatus.accelerationX = databuf[0];
    AccStatus.accelerationY = databuf[1];
    AccStatus.accelerationZ = databuf[2];
    return &AccStatus;
}

float lTalSixAxisGetYawAngle(void)
{
    float databuf[9] = {0};
    jee_device_read(six_axis_sensor_dev, NULL, (void *)databuf, sizeof(databuf));
    OnReceive.yawAngle = databuf[8];
    return OnReceive.yawAngle;
}

float lTalSixAxisGetPithAngle(void)
{
    float databuf[9] = {0};
    jee_device_read(six_axis_sensor_dev, NULL, (void *)databuf, sizeof(databuf));
    OnReceive.pithAngle = databuf[6];
    return OnReceive.pithAngle;
}

float lTalSixAxisGetRollAngle(void)
{
    float databuf[9] = {0};
    jee_device_read(six_axis_sensor_dev, NULL, (void *)databuf, sizeof(databuf));
    OnReceive.rollAngle = databuf[7];
    return OnReceive.rollAngle;
}

float lTalSixAxisGetAccelerationX(void)
{
    float databuf[9] = {0};
    jee_device_read(six_axis_sensor_dev, NULL, (void *)databuf, sizeof(databuf));
    OnReceive.accelerationX = databuf[0];
    return OnReceive.accelerationX;
}

float lTalSixAxisGetAccelerationY(void)
{
    float databuf[9] = {0};
    jee_device_read(six_axis_sensor_dev, NULL, (void *)databuf, sizeof(databuf));
    OnReceive.accelerationY = databuf[1];
    return OnReceive.accelerationY;
}

float lTalSixAxisGetAccelerationZ(void)
{
    float databuf[9] = {0};
    jee_device_read(six_axis_sensor_dev, NULL, (void *)databuf, sizeof(databuf));
    OnReceive.accelerationZ = databuf[2];
    return OnReceive.accelerationZ;
}

float lTalSixAxisGetAngularVelocityX(void)
{
    float databuf[9] = {0};
    jee_device_read(six_axis_sensor_dev, NULL, (void *)databuf, sizeof(databuf));
    OnReceive.angularVelocityX = databuf[3];
    return OnReceive.angularVelocityX;
}

float lTalSixAxisGetAngularVelocityY(void)
{
    float databuf[9] = {0};
    jee_device_read(six_axis_sensor_dev, NULL, (void *)databuf, sizeof(databuf));
    OnReceive.angularVelocityY = databuf[4];
    return OnReceive.angularVelocityY;
}

float lTalSixAxisGetAngularVelocityZ(void)
{
    float databuf[9] = {0};
    jee_device_read(six_axis_sensor_dev, NULL, (void *)databuf, sizeof(databuf));
    OnReceive.angularVelocityZ = databuf[5];
    return OnReceive.angularVelocityZ;
}

SixAxisOnReceive_t *lTalSixAxisOnReceive(void)
{
    float databuf[9] = {0};
    jee_device_read(six_axis_sensor_dev, NULL, (void *)databuf, sizeof(databuf));
    OnReceive.yawAngle = databuf[8];
    OnReceive.pithAngle = databuf[6];
    OnReceive.rollAngle = databuf[7];
    OnReceive.accelerationX = databuf[0];
    OnReceive.accelerationY = databuf[1];
    OnReceive.accelerationZ = databuf[2];
    OnReceive.angularVelocityX = databuf[3];
    OnReceive.angularVelocityY = databuf[4];
    OnReceive.angularVelocityZ = databuf[5];
    return &OnReceive;
}




void vTalSixAxisInit(void)
{
    // 查找设备
    six_axis_sensor_dev = jee_device_find("six-axis_QMI8658");
    if (six_axis_sensor_dev == JEE_NULL)
    {
        printf("can not find sensor Model\n");
        return;
    }
    else
        printf("find sensor Model ok\n");

    // 打开设备
    jee_err_t result = JEE_EOK;
    result = jee_device_open(six_axis_sensor_dev, JEE_DEVICE_FLAG_RDONLY);
    if (result != JEE_EOK)
    {
        printf("can not open senor device\n");
        return;
    }
    else
        printf("open senor device ok\n");
}

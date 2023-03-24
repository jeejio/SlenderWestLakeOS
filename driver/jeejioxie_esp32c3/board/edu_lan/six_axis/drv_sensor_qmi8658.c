/*
 * Copyright (c) 2022, jeejio Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author         Notes
 * 2023-02-07     xuyuhu         the first version
 */

#ifndef __JEE_SENSOR_QMI8658_H__
#define __JEE_SENSOR_QMI8658_H__

#include "device.h"
#include "hal_sensor.h" //sesor IO模型驱动框架头文件
#include "qmi8658.h"       //厂家驱动头文件  :舵机 旋转180°
#include "string.h"
#include "auto_init.h"
#include "drv_sensor_qmi8658.h"


jee_size_t sensorQmi8658FetchData(struct jee_sensor_device *sensor, void *buf, jee_size_t len)
{
    if (buf == NULL)
        return 0;

    //printf("sensorQmi8658FetchData\n");
    vSixAsixGetAllData((float *)buf);
    return (strlen(buf) - 1);
}

jee_err_t sensorQmi8658Control(struct jee_sensor_device *sensor, int cmd, void *arg)
{
    printf("sensorQmi8658FetchData enter [cmd ] %d\n", cmd);

    switch (cmd)
    {
    case JEE_SENSOR_CTRL_SET_POWER:
    {
        int power_mode_type = (int )arg;
        switch (power_mode_type)
        {
        case JEE_SENSOR_POWER_NORMAL:
        {
            printf(" start : imu_init \n");
            imu_init();
            printf(" end   : imu_init \n");
        }
        break;
        case JEE_SENSOR_POWER_DOWN:
        {
        }
        break;
        default:
            break;
        }
    }

    break;

    default:
        break;
    }

    return JEE_EOK;
}

static struct jee_sensor_ops sensor_ops =
    {
        sensorQmi8658FetchData,
        sensorQmi8658Control};

int sensorQmi8658Init(const char *name, struct jee_sensor_config *cfg)
{
    jee_int8_t result;
    jee_sensor_t sensor_six_axis = JEE_NULL;

    /* sensor register */
    sensor_six_axis = pvPortMalloc(sizeof(struct jee_sensor_device));
    if (sensor_six_axis == JEE_NULL)
        return -1;

    memset(sensor_six_axis, 0, sizeof(struct jee_sensor_device));

    sensor_six_axis->info.type = JEE_SENSOR_CLASS_SIX_AXIS;
    sensor_six_axis->info.vendor = JEE_SENSOR_VENDOR_UNKNOWN;
    sensor_six_axis->info.model = "QMI8658";
    // sensor_six_axis->info.unit       = RT_SENSOR_UNIT_DCELSIUS;
    // sensor_six_axis->info.intf_type  = JEE_SENSOR_INTF_I2C;
    // sensor_six_axis->info.range_max  = SENSOR_TEMP_RANGE_MAX;
    // sensor_six_axis->info.range_min  = SENSOR_TEMP_RANGE_MIN;

    if (cfg != JEE_NULL)
        memcpy(&sensor_six_axis->config, cfg, sizeof(struct jee_sensor_config));

    sensor_six_axis->ops = &sensor_ops;

    result = jee_hw_sensor_register(sensor_six_axis, name, JEE_DEVICE_FLAG_RDONLY, JEE_NULL);
    if (result != JEE_EOK)
    {
        // LOG_E("device register err code: %d", result);
        goto __exit;
    }

    return JEE_EOK;

__exit:
    if (sensor_six_axis)
        vPortFree(sensor_six_axis);

    return -JEE_ERROR;
}

#define LEDC_DEV_Name "qmi8658_dev"
int jeeHwSensorQmi8658Init()
{
    //printf("begin to execute jeeHwSensorQmi8658Init\n");

    struct jee_sensor_config cfg = {0};
    cfg.intf.dev_name = LEDC_DEV_Name;
    sensorQmi8658Init("QMI8658", &cfg);

    //printf("finish to execute jeeHwSensorQmi8658Init\n");
    return JEE_EOK;
}

INIT_DEVICE_EXPORT(jeeHwSensorQmi8658Init);

#endif

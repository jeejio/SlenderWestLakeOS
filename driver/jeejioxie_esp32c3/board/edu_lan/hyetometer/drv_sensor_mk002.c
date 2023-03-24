/*
 * Copyright (c) 2022, jeejio Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author         Notes
 * 2023-02-10     xuyuhu         the first version
 */

#ifndef __JEE_SENSOR_MK002_H__
#define __JEE_SENSOR_MK002_H__

#include "device.h"
#include "hal_sensor.h" //sesor IO模型驱动框架头文件
#include "mk002.h"      //厂家驱动头文件  :舵机 旋转180°
#include "string.h"
#include "auto_init.h"
#include "drv_sensor_mk002.h"

#define RFID_SET_CARD_DATA JEE_SENSOR_CTRL_USER_CMD_START + 1 /*cmd :向卡片写数据*/

jee_size_t sensorMk002FetchData(struct jee_sensor_device *sensor, void *buf, jee_size_t len)
{
    if (buf == NULL)
        return 0;

    printf("sensorMk002FetchData\n");
    hyetometer_get_info(buf);
    return (strlen(buf) - 1);
}

jee_err_t sensorMk002Control(struct jee_sensor_device *sensor, int cmd, void *arg)
{
    printf("sensorMk002Control enter [cmd ] %d\n", cmd);

    switch (cmd)
    {
    case JEE_SENSOR_CTRL_SET_POWER:
    {
        int power_mode_type = (int)arg;
        switch (power_mode_type)
        {
        case JEE_SENSOR_POWER_NORMAL:
        {
            printf(" start : hyetometer_init \n");
            hyetometer_init();
            printf(" end   : hyetometer_init \n");
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
        sensorMk002FetchData,
        sensorMk002Control};

int sensorMk002Init(const char *name, struct jee_sensor_config *cfg)
{
    jee_int8_t result;
    jee_sensor_t sensor_hyetometer = JEE_NULL;

    /* sensor register */
    sensor_hyetometer = pvPortMalloc(sizeof(struct jee_sensor_device));
    if (sensor_hyetometer == JEE_NULL)
        return -1;

    memset(sensor_hyetometer, 0, sizeof(struct jee_sensor_device));

    sensor_hyetometer->info.type = JEE_SENSOR_CLASS_HYETOMETER;
    sensor_hyetometer->info.vendor = JEE_SENSOR_VENDOR_UNKNOWN;
    sensor_hyetometer->info.model = "MK002";
    // sensor_hyetometer->info.unit       = RT_SENSOR_UNIT_DCELSIUS;
    // sensor_hyetometer->info.intf_type  = JEE_SENSOR_INTF_I2C;
    // sensor_hyetometer->info.range_max  = SENSOR_TEMP_RANGE_MAX;
    // sensor_hyetometer->info.range_min  = SENSOR_TEMP_RANGE_MIN;

    if (cfg != JEE_NULL)
        memcpy(&sensor_hyetometer->config, cfg, sizeof(struct jee_sensor_config));

    sensor_hyetometer->ops = &sensor_ops;

    result = jee_hw_sensor_register(sensor_hyetometer, name, JEE_DEVICE_FLAG_RDONLY, JEE_NULL);
    if (result != JEE_EOK)
    {
        // LOG_E("device register err code: %d", result);
        goto __exit;
    }

    return JEE_EOK;

__exit:
    if (sensor_hyetometer)
        vPortFree(sensor_hyetometer);

    return -JEE_ERROR;
}

#define ADC_DEV_Name "ADC_hard"
int jeeHwSensorMk002Init()
{
    //printf("begin to execute jeeHwSensorMk002Init\n");

    struct jee_sensor_config cfg = {0};
    cfg.intf.dev_name = ADC_DEV_Name;
    sensorMk002Init("MK002", &cfg);

    //printf("finish to execute jeeHwSensorMk002Init\n");
    return JEE_EOK;
}

INIT_DEVICE_EXPORT(jeeHwSensorMk002Init);

#endif

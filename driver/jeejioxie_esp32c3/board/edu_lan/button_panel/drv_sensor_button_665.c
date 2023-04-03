/*
 * Copyright (c) 2022, jeejio Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author         Notes
 * 2023-02-10     xuyuhu         the first version
 */

#ifndef __JEE_SENSOR_BUTTON_665_H__
#define __JEE_SENSOR_BUTTON_665_H__

#include "device.h"
#include "hal_sensor.h" //sesor IO模型驱动框架头文件
#include "button_665.h"       //厂家驱动头文件
#include "string.h"
#include "auto_init.h"

jee_size_t sensorButton665FetchData(struct jee_sensor_device *sensor, void *buf, jee_size_t len)
{
    if (buf == NULL)
        return 0;
    button_panel_get_button_info((int * )buf);
    return (strlen(buf) - 1);
}

jee_err_t sensorButton665Control(struct jee_sensor_device *sensor, int cmd, void *arg)
{
    printf("sensorButton665Control enter [cmd ] %d\n", cmd);

    switch (cmd)
    {
    case JEE_SENSOR_CTRL_SET_POWER:
    {
        int power_mode_type = (int)arg;
        switch (power_mode_type)
        {
        case JEE_SENSOR_POWER_NORMAL:
        {
            printf(" start : button_panel_init \n");
            button_panel_init();
            printf(" end   : button_panel_init \n");
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
        sensorButton665FetchData,
        sensorButton665Control};

int sensorButton665Init(const char *name, struct jee_sensor_config *cfg)
{
    jee_int8_t result;
    jee_sensor_t sensor = JEE_NULL;

    /* sensor register */
    sensor = pvPortMalloc(sizeof(struct jee_sensor_device));
    if (sensor == JEE_NULL)
        return -1;

    memset(sensor, 0, sizeof(struct jee_sensor_device));

    sensor->info.type = JEE_SENSOR_CLASS_BUTTON;
    sensor->info.vendor = JEE_SENSOR_VENDOR_UNKNOWN;
    sensor->info.model = "BT665";
    // sensor->info.unit       = RT_SENSOR_UNIT_DCELSIUS;
    // sensor->info.intf_type  = JEE_SENSOR_INTF_I2C;
    // sensor->info.range_max  = SENSOR_TEMP_RANGE_MAX;
    // sensor->info.range_min  = SENSOR_TEMP_RANGE_MIN;

    if (cfg != JEE_NULL)
        memcpy(&sensor->config, cfg, sizeof(struct jee_sensor_config));

    sensor->ops = &sensor_ops;

    result = jee_hw_sensor_register(sensor, name, JEE_DEVICE_FLAG_RDONLY, JEE_NULL);
    if (result != JEE_EOK)
    {
        // LOG_E("device register err code: %d", result);
        goto __exit;
    }

    return JEE_EOK;

__exit:
    if (sensor)
        vPortFree(sensor);

    return -JEE_ERROR;
}

#define ADC_DEV_NAME "ADC_BUTTON"
int jeeHwSensorButton665Init()
{
    //printf("begin to execute jeeHwSensorButton665Init\n");

    struct jee_sensor_config cfg = {0};
    cfg.intf.dev_name = ADC_DEV_NAME;
    sensorButton665Init("BT665", &cfg);
    //printf("finish to execute jeeHwSensorButton665Init\n");
    return JEE_EOK;
}

INIT_DEVICE_EXPORT(jeeHwSensorButton665Init);

#endif

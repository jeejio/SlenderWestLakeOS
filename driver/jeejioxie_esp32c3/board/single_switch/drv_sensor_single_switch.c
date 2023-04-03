/*
 * Copyright (c) 2022, jeejio Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author         Notes
 * 2023-03-23     xuyuhu         the first version
 */

#ifndef __JEE_SENSOR_SINGLE_SWITCH_H__
#define __JEE_SENSOR_SINGLE_SWITCH_H__

#include "device.h"
#include "hal_sensor.h"        //sesor IO模型驱动框架头文件
#include "single_switch.h"      //厂家驱动头文件  :单键开关
#include "string.h"
#include "auto_init.h"
#include "drv_sensor_single_switch.h"

#define SINGLE_SWITCH_CLEAN_BUTTON JEE_SENSOR_CTRL_USER_CMD_START + 1 


jee_size_t sensorSingleSwitchFetchData(struct jee_sensor_device *sensor, void *buf, jee_size_t len)
{
    if (buf == NULL)
        return 0;

    single_switch_get_info((int *)buf);
    return (strlen(buf) - 1);
}

jee_err_t sensorSingleSwitchControl(struct jee_sensor_device *sensor, int cmd, void *arg)
{
    switch (cmd)
    {
    case JEE_SENSOR_CTRL_SET_POWER:
    {
        int power_mode_type = (int)arg;
        switch (power_mode_type)
        {
        case JEE_SENSOR_POWER_NORMAL:
        {
            printf(" start : single_switch_init \n");
            single_switch_init();
            printf(" end   : single_switch_init \n");
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
    case SINGLE_SWITCH_CLEAN_BUTTON:
    {
        single_switch_clean_mode();
    }
    break;

    default:
        break;
    }

    return JEE_EOK;
}

static struct jee_sensor_ops sensor_ops =
    {
        sensorSingleSwitchFetchData,
        sensorSingleSwitchControl};

int sensorSingleSwitchInit(const char *name, struct jee_sensor_config *cfg)
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
    sensor->info.model = "switch0";

    if (cfg != JEE_NULL)
        memcpy(&sensor->config, cfg, sizeof(struct jee_sensor_config));

    sensor->ops = &sensor_ops;

    result = jee_hw_sensor_register(sensor, name, JEE_DEVICE_FLAG_RDONLY, JEE_NULL);
    if (result != JEE_EOK)
    {
        printf("result != JEE_EOK \n");
        goto __exit;
    }

    return JEE_EOK;

__exit:
    if (sensor)
        vPortFree(sensor);

    return -JEE_ERROR;
}

#define GPIO_DEV_Name "GPIO_Switch"
int jeeHwSensorSingleSwitchInit()
{
    struct jee_sensor_config cfg = {0};
    cfg.intf.dev_name = GPIO_DEV_Name;
    sensorSingleSwitchInit("SWITCH", &cfg);
    return JEE_EOK;
}
INIT_DEVICE_EXPORT(jeeHwSensorSingleSwitchInit);
#endif

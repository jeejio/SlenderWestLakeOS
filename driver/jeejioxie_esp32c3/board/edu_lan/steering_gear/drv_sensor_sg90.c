/*
 * Copyright (c) 2022, jeejio Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author         Notes
 * 2023-02-01     xuyuhu         the first version
 */

#ifndef __JEE_SENSOR_SG90_H__
#define __JEE_SENSOR_SG90_H__

#include "device.h"
#include "hal_sensor.h" //sesor IO模型驱动框架头文件
#include "sg90.h"       //厂家驱动头文件  :舵机 旋转180°
#include "string.h"
#include "auto_init.h"
#include "drv_sensor_sg90.h"

#define SG90_SET_ANGLE JEE_SENSOR_CTRL_USER_CMD_START + 1    /*cmd : 设置旋转角度*/
#define SG90_SET_DIRCTION JEE_SENSOR_CTRL_USER_CMD_START + 2 /*cmd : 设置旋转方向*/

jee_size_t sensorSg90FetchData(struct jee_sensor_device *sensor, void *buf, jee_size_t len)
{
    if (buf == NULL)
        return 0;

    printf("sg90_sensor_fetch_data\n");
    steeringGearGetDirctionAngle((uint32_t *)buf);
    return (strlen(buf) - 1);
}

jee_err_t sensorSg90Control(struct jee_sensor_device *sensor, int cmd, void *arg)
{
    printf("sg90_sensor_control enter [cmd ] %d\n", cmd);

    switch (cmd)
    {
    case SG90_SET_ANGLE:
    {
        steeringGearSetAngle(*(int *)arg);
    }
    break;
    case SG90_SET_DIRCTION:
    {
        steeringGearSetDirction(*(int *)arg);
    }
    break;
    case JEE_SENSOR_CTRL_SET_POWER:
    {
        int power_mode_type = (int )arg;
        switch (power_mode_type)
        {
        case JEE_SENSOR_POWER_NORMAL:
        {
            printf(" start : sg90Init \n");
            sg90Init();
            printf(" end   : sg90Init \n");
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
        sensorSg90FetchData,
        sensorSg90Control};

int sensorSg90Init(const char *name, struct jee_sensor_config *cfg)
{
    jee_int8_t result;
    jee_sensor_t sensor_steering_gear = JEE_NULL;

    /* sensor register */
    sensor_steering_gear = pvPortMalloc(sizeof(struct jee_sensor_device));
    if (sensor_steering_gear == JEE_NULL)
        return -1;

    memset(sensor_steering_gear, 0, sizeof(struct jee_sensor_device));

    sensor_steering_gear->info.type = JEE_SENSOR_CLASS_SG;
    sensor_steering_gear->info.vendor = JEE_SENSOR_VENDOR_UNKNOWN;
    sensor_steering_gear->info.model = "SG90";
    // sensor_steering_gear->info.unit       = RT_SENSOR_UNIT_DCELSIUS;
    // sensor_steering_gear->info.intf_type  = JEE_SENSOR_INTF_I2C;
    // sensor_steering_gear->info.range_max  = SENSOR_TEMP_RANGE_MAX;
    // sensor_steering_gear->info.range_min  = SENSOR_TEMP_RANGE_MIN;

    if (cfg != JEE_NULL)
        memcpy(&sensor_steering_gear->config, cfg, sizeof(struct jee_sensor_config));

    sensor_steering_gear->ops = &sensor_ops;

    result = jee_hw_sensor_register(sensor_steering_gear, name, JEE_DEVICE_FLAG_RDONLY, JEE_NULL);
    if (result != JEE_EOK)
    {
        // LOG_E("device register err code: %d", result);
        goto __exit;
    }

    return JEE_EOK;

__exit:
    if (sensor_steering_gear)
        vPortFree(sensor_steering_gear);

    return -JEE_ERROR;
}

#define LEDC_DEV_Name "lcdc_timer0hard"
int jeeHwSensorSg90Init()
{
    //printf("begin to execute jeeHwSensorSg90Init\n");

    struct jee_sensor_config cfg = {0};
    cfg.intf.dev_name = LEDC_DEV_Name;
    sensorSg90Init("SG90", &cfg);

    //printf("finish to execute jeeHwSensorSg90Init\n");
    return JEE_EOK;
}

INIT_DEVICE_EXPORT(jeeHwSensorSg90Init);

#endif

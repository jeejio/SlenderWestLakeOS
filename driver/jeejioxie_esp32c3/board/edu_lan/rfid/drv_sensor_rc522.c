/*
 * Copyright (c) 2022, jeejio Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author         Notes
 * 2023-02-01     xuyuhu         the first version
 */

#ifndef __JEE_SENSOR_RC522_H__
#define __JEE_SENSOR_RC522_H__

#include "device.h"
#include "hal_sensor.h" //sesor IO模型驱动框架头文件
#include "rc522.h"      //厂家驱动头文件  :舵机 旋转180°
#include "string.h"
#include "auto_init.h"
#include "drv_sensor_rc522.h"

#define RFID_SET_CARD_DATA JEE_SENSOR_CTRL_USER_CMD_START + 1 /*cmd :向卡片写数据*/

jee_size_t sensorRc522FetchData(struct jee_sensor_device *sensor, void *buf, jee_size_t len)
{
    if (buf == NULL)
        return 0;

    //printf("sensorRc522FetchData\n");
    rfid_get_card_info(buf);
    return (strlen(buf) - 1);
}

jee_err_t sensorRc522Control(struct jee_sensor_device *sensor, int cmd, void *arg)
{
    //printf("sensorRc522Control enter [cmd ] %d\n", cmd);

    switch (cmd)
    {
    case RFID_SET_CARD_DATA:
    {
        rfid_set_card_data((uint8_t *)arg);
    }
    break;
    case JEE_SENSOR_CTRL_SET_POWER:
    {
        int power_mode_type = (int)arg;
        switch (power_mode_type)
        {
        case JEE_SENSOR_POWER_NORMAL:
        {
            printf(" start : RCC522_Init \n");
            RCC522_Init();
            printf(" end   : RCC522_Init \n");
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
        sensorRc522FetchData,
        sensorRc522Control};

int sensorRc522Init(const char *name, struct jee_sensor_config *cfg)
{
    jee_int8_t result;
    jee_sensor_t sensor_rfid = JEE_NULL;

    /* sensor register */
    sensor_rfid = pvPortMalloc(sizeof(struct jee_sensor_device));
    if (sensor_rfid == JEE_NULL)
        return -1;

    memset(sensor_rfid, 0, sizeof(struct jee_sensor_device));

    sensor_rfid->info.type = JEE_SENSOR_CLASS_RFID;
    sensor_rfid->info.vendor = JEE_SENSOR_VENDOR_UNKNOWN;
    sensor_rfid->info.model = "RC522";
    // sensor_rfid->info.unit       = RT_SENSOR_UNIT_DCELSIUS;
    // sensor_rfid->info.intf_type  = JEE_SENSOR_INTF_I2C;
    // sensor_rfid->info.range_max  = SENSOR_TEMP_RANGE_MAX;
    // sensor_rfid->info.range_min  = SENSOR_TEMP_RANGE_MIN;

    if (cfg != JEE_NULL)
        memcpy(&sensor_rfid->config, cfg, sizeof(struct jee_sensor_config));

    sensor_rfid->ops = &sensor_ops;

    result = jee_hw_sensor_register(sensor_rfid, name, JEE_DEVICE_FLAG_RDONLY, JEE_NULL);
    if (result != JEE_EOK)
    {
        // LOG_E("device register err code: %d", result);
        goto __exit;
    }

    return JEE_EOK;

__exit:
    if (sensor_rfid)
        vPortFree(sensor_rfid);

    return -JEE_ERROR;
}

#define SPI_DEV_Name "spi_hard"
int jeeHwSensorRc522Init()
{
    //printf("begin to execute jeeHwSensorRc522Init\n");

    struct jee_sensor_config cfg = {0};
    cfg.intf.dev_name = SPI_DEV_Name;
    sensorRc522Init("RC522", &cfg);

    //printf("finish to execute jeeHwSensorRc522Init\n");
    return JEE_EOK;
}

INIT_DEVICE_EXPORT(jeeHwSensorRc522Init);

#endif

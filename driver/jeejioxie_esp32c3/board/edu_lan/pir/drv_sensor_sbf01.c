/*
 * Copyright (c) 2023, jeejio Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author         Notes
 * 2023-01-31     huzhiqiang     the first version
 */

#include "device.h"
#include "hal_sensor.h" //sesor IO模型驱动框架头文件
#include "sbf01.h"      //厂家驱动头文件
#include "string.h"
#include "auto_init.h"
#include "hal_gpio.h"
#include "esp_log.h"
#include "drv_sensor_sbf01.h"

#define SENSOR_PIR_NAME "pir"

/**
 * @brief       读取PIR状态
 *
 * NOTE:        -
 *
 * @param[in]   sensor    - 传感器指针
 * @param[out]  buf      - 出参指针，读取的状态通过该指针带出
 * @param[in]   len      - 入参指针，指示入参指针的数据长度
 * @return      固定为0
 *
 */
jee_size_t sensorPirReadStatus(struct jee_sensor_device *sensor, void *buf, jee_size_t len)
{
    uint8_t *pbuf = buf;
    if (buf == NULL)
        return 0;
    pbuf[0] = lSbf01ReadStatus();
    return 0;
}

/**
 * @brief       sensor指令处理函数
 *
 * NOTE:        -
 *
 * @param[in]   sensor    - 传感器指针
 * @param[in]   cmd       - 命令，用于制定对传感器的操作类型
 * @param[in]   arg       - 入参指针，用于传入各种类型的数据
 * @return      固定为JEE_EOK
 *
 */
jee_err_t sensorPirControl(struct jee_sensor_device *sensor, int cmd, void *arg)
{
    switch (cmd)
    {
    case SENSOR_PIR_INIT:
    {
        printf("sensor pir init\n");
    }
    break;
    case JEE_SENSOR_CTRL_SET_POWER:
    {
        int power_mode_type = (int)arg;
        switch (power_mode_type)
        {
        case JEE_SENSOR_POWER_NORMAL:
        {
            printf(" start : vSbf01Init \n");
            vSbf01Init();
            printf(" end   : vSbf01Init \n");
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
    }
    return JEE_EOK;
}

static struct jee_sensor_ops sensor_ops =
{
    sensorPirReadStatus,
    sensorPirControl
};

 /**
   * @brief       sensor框架注册
   *
   * NOTE:        -
   * 
   * @param[in]   sensor    - 设备名称
   * @param[in]   cfg       - 传感器的配置参数
   * @return      返回sensor框架注册结果
   *
   */
int sensorPirInit(const char *name, struct jee_sensor_config *cfg)
{
    jee_int8_t result;
    jee_sensor_t sensor_pir = JEE_NULL;
    /* sensor register */
    sensor_pir = pvPortMalloc(sizeof(struct jee_sensor_device));
    if (sensor_pir == JEE_NULL)
        return -1;
    memset(sensor_pir, 0, sizeof(struct jee_sensor_device));
    sensor_pir->info.type = JEE_SENSOR_CLASS_PIR;
    sensor_pir->info.vendor = JEE_SENSOR_VENDOR_UNKNOWN;
    sensor_pir->info.model = "pir0";
    // sensor_pir->info.intf_type = JEE_SENSOR_INTF_I2C;

    if (cfg != JEE_NULL)
        memcpy(&sensor_pir->config, cfg, sizeof(struct jee_sensor_config));

    sensor_pir->ops = &sensor_ops;

    result = jee_hw_sensor_register(sensor_pir, name, JEE_DEVICE_FLAG_RDWR, JEE_NULL);
    if (result != JEE_EOK)
    {
        // LOG_E("device register err code: %d", result);
        goto __exit;
    }

    return JEE_EOK;

__exit:
    if (sensor_pir)
        vPortFree(sensor_pir);

    return -JEE_ERROR;
}

 /**
   * @brief       sensor初始化
   *
   * NOTE:        -
   * 
   * @return      固定为JEE_EOK
   *
   */
int jeeHwSensorPirInit()
{
    //printf("begin to execute jee_hw_sensor_PIR_init\n");
    struct jee_sensor_config cfg = {0};
    cfg.intf.dev_name = SENSOR_PIR_NAME;
    sensorPirInit("PIR", &cfg);
    //printf("finish to execute jee_hw_sensor_PIR_init\n");
    return JEE_EOK;
}

INIT_DEVICE_EXPORT(jeeHwSensorPirInit);

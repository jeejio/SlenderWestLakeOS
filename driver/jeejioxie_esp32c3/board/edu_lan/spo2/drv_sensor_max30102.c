/*
 * Copyright (c) 2022, jeejio Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author         Notes
 * 2023-02-01     xuyuhu         the first version
 */

#include "device.h"
#include "hal_sensor.h" //sesor IO模型驱动框架头文件
#include "max30102.h"   //厂家驱动头文件
#include "string.h"
#include "auto_init.h"
#include "drv_sensor_max30102.h"

#define MAX30102_SET_ONOFFCMD (JEE_SENSOR_CTRL_USER_CMD_START + 1) /*cmd : */

 /**
   * @brief       读取血氧感器状态
   *
   * NOTE:        -
   * 
   * @param[in]   sensor    - 传感器指针
   * @param[out]  buf      - 出参指针，读取的状态通过该指针带出
   * @param[in]   len      - 入参指针，指示入参指针的数据长度
   * @return      固定为0
   *
   */
jee_size_t sensorFetchData(struct jee_sensor_device *sensor, void *buf, jee_size_t len)
{

    jee_int32_t *pbuf = buf;

    if (buf == NULL)
        return 0;
    jee_int32_t spo2Data, heartRace;
    bloodoxygen_sensor_get_data(&spo2Data, &heartRace);
    pbuf[0] = bloodoxygen_get_onoff();
    pbuf[1] = bloodoxygen_get_send_flag();
    pbuf[2] = spo2Data;
    pbuf[3] = heartRace;
    return (0);
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
jee_err_t sensorControl(struct jee_sensor_device *sensor, int cmd, void *arg)
{
    printf("Max30102_sensor_control enter [cmd ] %d\n", cmd);

    switch (cmd)
    {
    case MAX30102_SET_ONOFFCMD:
    {
        bloodoxygen_set_onoff(*(jee_uint8_t *)arg);
    }
    break;

        break;
    case JEE_SENSOR_CTRL_SET_POWER:
    {
        int power_mode_type = (int)arg;
        switch (power_mode_type)
        {
        case JEE_SENSOR_POWER_NORMAL:
        {
            printf(" start : Max30102 \n");
            bloodoxygen_sensor_init();
            printf(" end   : Max30102 \n");
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
        sensorFetchData,
        sensorControl};

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
int sensorMax30102Init(const char *name, struct jee_sensor_config *cfg)
{
    jee_int8_t result;
    jee_sensor_t sensor_spo2 = JEE_NULL;

    /* sensor register */
    sensor_spo2 = pvPortMalloc(sizeof(struct jee_sensor_device));
    if (sensor_spo2 == JEE_NULL)
        return -1;

    memset(sensor_spo2, 0, sizeof(struct jee_sensor_device));

    sensor_spo2->info.type = JEE_SENSOR_CLASS_SPO2;
    sensor_spo2->info.vendor = JEE_SENSOR_VENDOR_UNKNOWN;
    sensor_spo2->info.model = "max30102";
    // sensor_steering_gear->info.unit       = RT_SENSOR_UNIT_DCELSIUS;
    // sensor_steering_gear->info.intf_type  = JEE_SENSOR_INTF_I2C;
    // sensor_steering_gear->info.range_max  = SENSOR_TEMP_RANGE_MAX;
    // sensor_steering_gear->info.range_min  = SENSOR_TEMP_RANGE_MIN;

    if (cfg != JEE_NULL)
        memcpy(&sensor_spo2->config, cfg, sizeof(struct jee_sensor_config));

    sensor_spo2->ops = &sensor_ops;

    result = jee_hw_sensor_register(sensor_spo2, name, JEE_DEVICE_FLAG_RDONLY, JEE_NULL);
    if (result != JEE_EOK)
    {
        // LOG_E("device register err code: %d", result);
        goto __exit;
    }

    return JEE_EOK;

__exit:
    if (sensor_spo2)
        vPortFree(sensor_spo2);

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
#define LEDC_DEV_Name "iic0"
int jeeHwSensorMax30102Init()
{
    //printf("begin to execute jeeHwSensorMax30102Init\n");

    struct jee_sensor_config cfg = {0};
    cfg.intf.dev_name = LEDC_DEV_Name;
    sensorMax30102Init("max30102", &cfg);

    //printf("finish to execute jeeHwSensorMax30102Init\n");
    return JEE_EOK;
}

INIT_DEVICE_EXPORT(jeeHwSensorMax30102Init);

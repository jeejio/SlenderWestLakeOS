/*
 * Copyright (c) 2022, jeejio Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author         Notes
 * 2023-1-31     QiuCuiTang      the first version
 */


#include "device.h"
#include "hal_sensor.h" //sesor IO模型驱动框架头文件
#include "ws2835.h"     //厂家驱动头文件
#include "string.h"
#include "auto_init.h"
#include "driver/gpio.h"
#include "drv_sensor_ws2835.h"

 /**
   * @brief       读取LED矩阵感器状态
   *
   * NOTE:        -
   * 
   * @param[in]   sensor    - 传感器指针
   * @param[out]  buf      - 出参指针，读取的状态通过该指针带出
   * @param[in]   len      - 入参指针，指示入参指针的数据长度
   * @return      固定为0
   *
   */
static jee_size_t sensorFetchData(struct jee_sensor_device *sensor, void *buf, jee_size_t len)
{
    if (buf == NULL)
        return 0;

    return (1);
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
static jee_err_t sensorControl(struct jee_sensor_device *sensor, int cmd, void *arg)
{
    printf("ws2812 sensorControl enter [cmd ] %d\n", cmd);

    switch (cmd)
    {
    case WS2835_PIXEL_SET_CMD:
    {
        jee_uint32_t *data = arg;
        ws2812SetPixel(data[0], data[1], data[2], data[3]);
    }
    break;
    case WS2835_REFRESH_SET_CMD:
    {

        ws2812Refresh(*(jee_uint32_t *)arg);
    }
    break;
    case WS2835_CLEAR_SET_CMD:
    {
        ws2812Clear(*(jee_uint32_t *)arg);
    }
    break;
    case WS2835_DEL_SET_CMD:
    {
        ws2812Del();
    }
    break;
    case JEE_SENSOR_CTRL_SET_POWER:
    {
        int power_mode_type = (int)arg;
        switch (power_mode_type)
        {
        case JEE_SENSOR_POWER_NORMAL:
        {
            printf(" start : ws2812Init \n");
            ws2812Init();
            printf(" end   : ws2812Init \n");
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
int sensorWs2835Init(const char *name, struct jee_sensor_config *cfg)
{
    jee_int8_t result;
    jee_sensor_t sensor_Magnetometer = JEE_NULL;

    /* sensor register */
    sensor_Magnetometer = pvPortMalloc(sizeof(struct jee_sensor_device));
    if (sensor_Magnetometer == JEE_NULL)
        return -1;

    memset(sensor_Magnetometer, 0, sizeof(struct jee_sensor_device));

    sensor_Magnetometer->info.type = JEE_SENSOR_CLASS_LEDARRAY;
    sensor_Magnetometer->info.vendor = JEE_SENSOR_VENDOR_UNKNOWN;
    sensor_Magnetometer->info.model = "ws2835";
    sensor_Magnetometer->info.intf_type = JEE_SENSOR_INTF_ONEWIRE;

    if (cfg != JEE_NULL)
        memcpy(&sensor_Magnetometer->config, cfg, sizeof(struct jee_sensor_config));

    sensor_Magnetometer->ops = &sensor_ops;

    result = jee_hw_sensor_register(sensor_Magnetometer, name, JEE_DEVICE_FLAG_RDONLY, JEE_NULL);
    if (result != JEE_EOK)
    {
        // LOG_E("device register err code: %d", result);
        goto __exit;
    }

    return JEE_EOK;

__exit:
    if (sensor_Magnetometer)
        vPortFree(sensor_Magnetometer);

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
int jeeHwSensorWs2835Init()
{
    //printf("begin to execute jeeHwSensorWs2835Init\n");
    // i2c_master_init();

    struct jee_sensor_config cfg = {0};
    cfg.intf.dev_name = "rmt0";
    cfg.intf.user_data = (void *)0;
    sensorWs2835Init("ws2835", &cfg);

    //printf("finish to execute jeeHwSensorWs2835Init\n");
    return JEE_EOK;
}

INIT_DEVICE_EXPORT(jeeHwSensorWs2835Init);



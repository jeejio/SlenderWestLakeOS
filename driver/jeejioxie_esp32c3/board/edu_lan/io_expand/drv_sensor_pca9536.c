/*
 * Copyright (c) 2022, jeejio Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author         Notes
 * 2023-2-1     QiuCuiTang      the first version
 */



#include "device.h"
#include "hal_sensor.h" //sesor IO模型驱动框架头文件
#include "pca9536.h"    //厂家驱动头文件
#include "string.h"
#include "auto_init.h"
#include "driver/gpio.h"
#include "drv_sensor_pca9536.h"


 /**
   * @brief       读取io拓展芯片相关数据
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
    int32_t *pbuf = buf;

    if (buf == NULL)
        return 0;

    pbuf[0] = pca9536_read_io();

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
    printf("pca9536Init sensor_control enter [cmd ] %d\n", cmd);

    switch (cmd)
    {

    case JEE_SENSOR_CTRL_SET_POWER:
    {
        int power_mode_type = (int)arg;
        switch (power_mode_type)
        {
        case JEE_SENSOR_POWER_NORMAL:
        {
            printf(" start : pca9536Init \n");
            pca9536Init();
            printf(" end   : pca9536Init \n");
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
int sensorPca9536Init(const char *name, struct jee_sensor_config *cfg)
{
    jee_int8_t result;
    jee_sensor_t sensor_WaterPump = JEE_NULL;

    /* sensor register */
    sensor_WaterPump = pvPortMalloc(sizeof(struct jee_sensor_device));
    if (sensor_WaterPump == JEE_NULL)
        return -1;

    memset(sensor_WaterPump, 0, sizeof(struct jee_sensor_device));

    sensor_WaterPump->info.type = JEE_SENSOR_CLASS_IOEXPAND;
    sensor_WaterPump->info.vendor = JEE_SENSOR_VENDOR_UNKNOWN;
    sensor_WaterPump->info.model = "pca9536";
    sensor_WaterPump->info.intf_type = JEE_SENSOR_INTF_ONEWIRE;

    if (cfg != JEE_NULL)
        memcpy(&sensor_WaterPump->config, cfg, sizeof(struct jee_sensor_config));

    sensor_WaterPump->ops = &sensor_ops;

    result = jee_hw_sensor_register(sensor_WaterPump, name, JEE_DEVICE_FLAG_RDONLY, JEE_NULL);
    if (result != JEE_EOK)
    {
        // LOG_E("device register err code: %d", result);
        goto __exit;
    }

    
    return JEE_EOK;

__exit:
    if (sensor_WaterPump)
        vPortFree(sensor_WaterPump);

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
int jeeHwSensorPca9536Init()
{
    //printf("begin to execute jee_hw_sensor_cc6201st_init\n");
    // i2c_master_init();

    struct jee_sensor_config cfg = {0};
    cfg.intf.dev_name = "iic0";
    cfg.intf.user_data = (void *)0;
    sensorPca9536Init("pca9536", &cfg);

    //printf("finish to execute jee_hw_sensor_DHT20_init\n");
    return JEE_EOK;
}

INIT_DEVICE_EXPORT(jeeHwSensorPca9536Init);


